#include "robot/bounded_queue.hpp"
#include "robot/capture.hpp"
#include "robot/config.hpp"
#ifdef ROBOT_WITH_RKNN
#include "robot/detector.hpp"
#endif
#ifdef ROBOT_WITH_MPP
#include "robot/encoder.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;
std::atomic<bool> running{true};
static_assert(std::atomic<bool>::is_always_lock_free, "the signal handler needs a lock-free flag");
void request_stop(int) 
{ 
    running.store(false, std::memory_order_relaxed); 
}

class FrameWorker {
public:
    FrameWorker(std::size_t depth, std::function<void(robot::FramePtr)> consume)
        : queue_(depth), thread_([this, consume = std::move(consume)] {
              try {
                  robot::FramePtr frame;
                  while (queue_.pop(frame)) {
                      consume(std::move(frame));
                      completed_.fetch_add(1, std::memory_order_relaxed);
                  }
              } catch (...) {
                  {
                      std::lock_guard<std::mutex> lock(error_mutex_);
                      error_ = std::current_exception();
                  }
                  queue_.close();
              }
          }) {}

    ~FrameWorker() {
        queue_.close();
        if (thread_.joinable()) thread_.join();
    }

    void submit(const robot::FramePtr& frame) {
        if (!queue_.push_latest(frame)) check();
    }

    void check() const {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if (error_) std::rethrow_exception(error_);
    }

    void close() { queue_.close(); }

    void finish() {
        if (thread_.joinable()) thread_.join();
        check();
    }

    std::size_t dropped() const { return queue_.dropped(); }
    std::uint64_t completed() const { return completed_.load(std::memory_order_relaxed); }

private:
    robot::BoundedQueue<robot::FramePtr> queue_;
    mutable std::mutex error_mutex_;
    std::exception_ptr error_;
    std::atomic<std::uint64_t> completed_{0};
    std::thread thread_;
};


/* -------------------------------------------------------------------------------------------------------------------------------------*/
// 一个Config类型的对象，包含了程序运行所需的各种配置参数。这个函数的作用是根据这些配置参数来运行整个程序。
int run(const robot::Config& config) 
{
    // 根据配置参数判断是否需要进行目标检测和视频流处理
    // model参数为空表示不进行目标检测
    const bool detect = !config.model.empty();
    // h264_file和rtmp_url参数为空表示不进行视频流处理
    const bool stream = !config.h264_file.empty() || !config.rtmp_url.empty();
#ifndef ROBOT_WITH_RKNN
    if (detect) throw std::runtime_error("rebuild with -DROBOT_WITH_RKNN=ON to enable model");
#endif
#ifndef ROBOT_WITH_RGA
    if (config.use_rga) throw std::runtime_error("rebuild with -DROBOT_WITH_RGA=ON to enable use_rga");
#endif
#ifndef ROBOT_WITH_MPP
    if (stream) throw std::runtime_error("rebuild with -DROBOT_WITH_MPP=ON to encode or stream");
#endif
    // 创建一个Capture对象，用于从摄像头设备捕获视频帧。根据配置参数决定是否使用RGA进行图像缩放。
    // stream || config.use_rga 表示如果需要进行视频流处理或者使用RGA进行图像缩放，则将export_dma设置为true，否则为false。
    robot::Capture capture(config, stream || config.use_rga);
    // 获取摄像头的捕获格式信息，包括分辨率、帧率、缓冲区数量等，并打印这些信息到标准输出。
    const auto& format = capture.format();
    // 打印摄像头设备路径、分辨率、步幅、缓冲区数量、API类型、帧率以及是否支持DMA缓冲区导出等信息。
    std::cout << capture.device() << " NV12 " << format.width << 'x' << format.height
              << " stride=" << format.stride << " buffers=" << format.buffers
              << " api=" << (format.multiplanar ? "MPLANE" : "single-planar")
              << " fps=" << format.fps_num << '/' << format.fps_den
              << (format.fps_confirmed ? " (driver)" : " (requested, driver did not report frame interval)")
              << " dma_buf=" << (format.dma_exported ? "yes" : "no") << std::endl;

#ifdef ROBOT_WITH_RKNN
    // 如果启用了RKNN目标检测功能，则创建一个Detector对象用于处理视频帧
    std::unique_ptr<robot::Detector> detector;
#endif
#ifdef ROBOT_WITH_MPP
    // 如果启用了MPP视频流处理功能，则创建一个Stream对象用于处理视频帧
    std::unique_ptr<robot::Stream> encoder;
#endif
    // 创建两个FrameWorker对象，分别用于处理目标检测和视频流处理的任务。FrameWorker是一个线程安全的队列，用于在后台线程中处理视频帧。
    std::unique_ptr<FrameWorker> inference;
    std::unique_ptr<FrameWorker> streaming;
// 根据配置参数判断是否需要启用目标检测和视频流处理功能，并创建相应的对象和线程来处理视频帧。 
#ifdef ROBOT_WITH_RKNN
    // 如果启用了RKNN目标检测功能，则创建一个Detector对象用于处理视频帧。
    // 并创建一个FrameWorker对象用于在后台线程中处理视频帧并调用detector的process方法进行目标检测。
    if (detect) {
        // detector对象是一个智能指针，指向一个Detector对象，该对象负责处理视频帧并进行目标检测。
        detector = std::make_unique<robot::Detector>(config);
        // inference对象是一个智能指针，指向一个FrameWorker对象，该对象负责在后台线程中处理视频帧并调用detector的process方法进行目标检测。
        inference = std::make_unique<FrameWorker>(config.inference_queue, [&](robot::FramePtr frame) 
        {
            detector->process(std::move(frame));
        });
    }
#endif
// 根据配置参数判断是否需要启用视频流处理功能，并创建相应的对象和线程来处理视频帧。
#ifdef ROBOT_WITH_MPP
    // 如果启用了MPP视频流处理功能，则创建一个Stream对象用于处理视频帧。
    if (stream) {
        // encoder对象是一个智能指针，指向一个Stream对象，该对象负责处理视频帧并进行视频编码和流式传输。
        encoder = std::make_unique<robot::Stream>(config, format);
        // streaming对象是一个智能指针，指向一个FrameWorker对象，该对象负责在后台线程中处理视频帧并调用encoder的process方法进行视频编码和流式传输。
        streaming = std::make_unique<FrameWorker>(config.stream_queue, [&](robot::FramePtr frame) {
            encoder->process(std::move(frame));
        });
    }
#endif

    // 开始捕获视频帧。
    capture.start();
    // 记录总运行起始时间，用于程序结束时算平均 FPS。
    const auto start = Clock::now();
    // 记录上一次成功拿到帧的时间，用作超时看门狗。
    auto last_frame = start;
    // 记录上一次报告的时间点，用于计算每秒处理的视频帧数量。
    auto last_report = start;
    // 初始化一些计数器，用于统计处理的视频帧数量、上一次报告的帧数量、序列号间的间隔等信息。
    std::uint64_t frames = 0;
    std::uint64_t last_report_frames = 0;
    std::uint64_t sequence_gaps = 0;
    std::uint32_t previous = 0;
    // 默认在第 30 帧抓取一张图（如果总帧数 config.frames < 30 则取最后一帧）。
    // 之所以不等第 1 帧，通常是为了跳过摄像头刚启动时自动曝光（AE）/自动白平衡（AWB）尚未稳定的阶段。
    unsigned snapshot_at = 30;  // 默认第 30 帧抓拍快照
    // 如果配置文件中设置了 frames 参数，并且该参数的值大于 0 且小于 30，则将 snapshot_at 设置为 frames 的值。
    // frames这个参数表示的是程序总共采集多少帧视频后自动停止
    if (config.frames > 0 && config.frames < 30) {
        snapshot_at = config.frames;  
    }
    // 记录上一次报告的帧数量，用于计算每秒处理的视频帧数量。
    const auto report = [&](double fps) {
        std::cout << "frames=" << frames << " measured_fps=" << std::fixed << std::setprecision(2) << fps
                  << " v4l2_gaps=" << sequence_gaps
                  << " inference_frames=" << (inference ? inference->completed() : 0)
                  << " encoder_frames=" << (streaming ? streaming->completed() : 0)
                  << " inference_queue_drops=" << (inference ? inference->dropped() : 0)
                  << " stream_queue_drops=" << (streaming ? streaming->dropped() : 0) << std::endl;
    };

            // 如果没有收到推出信号
    while (running.load(std::memory_order_relaxed) &&
           // 如果配置了无限运行(config.frames == 0)或当前采到的 frames 还没达到目标上限。
           (config.frames == 0 || frames < static_cast<unsigned>(config.frames))) {
        // 主线程每运行一圈就检查一次 inference 和 streaming 的后台线程是否有异常抛出，如果有则重新抛出异常。 
        if (inference) inference->check();
        if (streaming) streaming->check();
        // 向摄像头要一帧画面，最多等 250 毫秒（0.25 秒）。
        auto frame = capture.next(250);
        const auto now = Clock::now();
        // 如果等了 250ms 没拿到画面呢？
        if (!frame) {
            // 检查距离上一次拿到画面（last_frame）是不是超过 5 秒了？如果满 5 秒一帧都没有，说明硬件/驱动肯定出问题了，直接抛错退出，
            // 并提示你去查 OV13850 排线、ISP 或摄像头是不是被别的程序占用了。
            if (now - last_frame > std::chrono::seconds(5)) {
                throw std::runtime_error("no valid frame for 5 seconds; check OV13850 links, ISP and camera ownership");
            }
            // 如果还没满 5 秒，就 continue 跳过后面的代码，再试下一轮。
            continue;
        }
        // 刷新“最近拿到帧的时间”（把 5 秒看门狗倒计时清零）。
        last_frame = now;

        /* 
            摄像头驱动会给每一帧打一个递增序号 frame->sequence（1, 2, 3, 4...）
            用当前序号减去上一帧序号得到 delta。正常紧挨着的两帧，delta 应该是 1。
        */ 
        const auto delta = frame->sequence - previous;
        /*
            如果上一帧是 10，这一帧直接跳到了 13，那 delta = 3，说明中间漏掉了 3 - 1 = 2 帧，加到 sequence_gaps 里。
            frames != 0 是为了跳过第 1 帧（第 1 帧还没有“上一帧”可比）；< 0x80000000U 是防止无符号整数减法溢出误判。
        */ 
        if (frames != 0 && delta > 1 && delta < 0x80000000U) sequence_gaps += delta - 1;
        // 更新上个帧的序号为当前帧的序号，方便下一轮循环计算 delta。 成功采集的总帧数加 1。
        previous = frame->sequence;
        ++frames;
        /*
            当帧数刚好数到前面算好的 snapshot_at（默认第 30 帧），而且你在配置文件里填了 snapshot 保存路径（!empty() 不为空）
            就把这一帧存成 NV12 图片文件，方便检查摄像头画面清不清楚、颜色正不正常。
        */
        if (frames == snapshot_at && !config.snapshot.empty()) {
            robot::save_nv12(frame->image, config.snapshot);
            std::cout << "saved " << config.snapshot << " as packed " << format.width << 'x'
                      << format.height << " NV12" << std::endl;
        }
        // 把刚拿到的这一帧 frame 塞进编码推流队列和 AI 推理队列里。
        if (streaming) streaming->submit(frame);
        if (inference) inference->submit(frame);
        /*
            ⑦ 每满 1 秒打印一次体检报告
        */
        // 先算一下当前时间 now 距离上次打印日志 last_report 过去了多少秒（seconds）。
        const auto seconds = std::chrono::duration<double>(now - last_report).count();
        if (seconds >= 1.0) {
            report(static_cast<double>(frames - last_report_frames) / seconds);
            last_report = now;
            last_report_frames = frames;
        }
    }
    // 在我们推出while循环后，立即结算采集总耗时。
    const auto seconds = std::chrono::duration<double>(Clock::now() - start).count();
    /*
        close() = 门口挂“停止迎客”牌：通知推理队列和推流队列“不会再有新视频帧进来了”。
        如果不调 close()，后台子线程处理完手头的帧后还会傻傻死等下一帧，程序就永远卡住退不出来。
        finish() = 等店里最后几桌客人吃完再锁门：主线程在这里阻塞等待，
        等后台的推理线程和推流线程把队列里残余的最后几帧全部算完、
        推完并安全退出线程（底层对应 std::thread::join()）。
    */
    if (inference) inference->close();
    if (streaming) streaming->close();
    if (inference) inference->finish();
    if (streaming) streaming->finish();
#ifdef ROBOT_WITH_MPP
    if (encoder) encoder->finish();
#endif
    // 最后再调用一次前面定义的 report(...)，传入 总帧数 ÷ 总秒数，打印整场运行的最终平均 FPS 和总丢帧统计。
    report(seconds > 0 ? static_cast<double>(frames) / seconds : 0);
    return 0;
}

}  // namespace

// argc这个参数的意思是命令行参数的个数，argv是一个指向字符串数组的指针，通过这个指针来访问我们在命令行中输入的参数。
int main(int argc, char** argv) {
    std::string_view first_argument = "";  // first_argument用来存储命令行中第一个参数的值，如果没有提供参数，则默认为空字符串。
    if (argc >= 2) {                       // argc >= 2表示命令行中至少有一个参数（argv[0]是程序的名称，argv[1]是第一个参数）
        first_argument = argv[1];          // 如果满足这个条件，就将第一个参数的值赋给first_argument。
    }
    // 如果第一个参数是"--help"或"-h"，或者命令行中有超过两个参数（程序名称和一个额外参数），则进入这个条件分支。
    if (first_argument == "--help" || first_argument == "-h" || argc > 2) { 
        // 如果满足条件，就打印出程序的使用说明，告诉用户如何正确运行程序。
        std::cout << "Usage: inspection_robot [config/robot.conf]\n";
        return argc > 2 ? 2 : 0;
    }
    // 设置了几个信号处理函数，用于在程序运行时捕获特定的信号，并执行相应的操作。
    // 当程序接收到SIGINT信号（通常是用户按下Ctrl+C）时，会调用request_stop函数来处理这个信号。
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    // 忽略SIGPIPE信号，这个信号通常在管道或套接字写入时发生，如果对方已经关闭连接。
    std::signal(SIGPIPE, SIG_IGN);
    try 
    {
        // 设置默认的配置文件路径为"config/robot.conf"。
        const char* config_path = "config/robot.conf";
        // 如果命令行中提供了一个参数（即argc == 2），则将config_path设置为用户提供的参数值。
        if (argc == 2) 
        {
            config_path = argv[1];
        }
        // 调用load_config函数加载配置文件，并将返回的Config对象存储在config变量中。
        const auto config = robot::load_config(config_path);
        return run(config);
    } 
    catch (const std::exception& error) 
    {
        // 如果在加载配置文件或运行程序时发生异常，会捕获该异常，并打印出错误信息到标准错误输出流（std::cerr）。
        std::cerr << "fatal: " << error.what() << '\n';
        return 1;
    }
}

