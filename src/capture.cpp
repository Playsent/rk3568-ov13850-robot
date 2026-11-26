#include "robot/capture.hpp"
#include "robot/unique_fd.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

namespace robot {
namespace {

int xioctl(int fd, unsigned long request, void* argument) {
    int result;
    do { result = ::ioctl(fd, request, argument); } while (result < 0 && errno == EINTR);
    return result;
}

void check(int result, const std::string& operation) {
    if (result < 0) throw std::system_error(errno, std::generic_category(), operation);
}

std::string resolve_device(const std::string& device) {
    if (device != "auto") return device;
    namespace fs = std::filesystem;
    const fs::path root("/sys/class/video4linux");
    std::vector<std::string> matches;
    if (fs::exists(root)) {
        for (const auto& node : fs::directory_iterator(root)) {
            const auto name = node.path().filename().string();
            if (name.rfind("video", 0) != 0) continue;
            std::ifstream label(node.path() / "name");
            std::string text;
            if (std::getline(label, text) && text == "rkisp_mainpath") {
                matches.push_back("/dev/" + name);
            }
        }
    }
    if (matches.size() == 1) return matches.front();
    if (matches.empty()) {
        throw std::runtime_error("no rkisp_mainpath found; inspect the camera with scripts/probe_camera.sh");
    }
    throw std::runtime_error("multiple rkisp_mainpath nodes; use media-ctl -p and set video_device explicitly");
}

struct Mapping {
    void* address{MAP_FAILED};
    std::size_t length{};
    UniqueFd dma_fd;

    ~Mapping() {
        if (address != MAP_FAILED) ::munmap(address, length);
    }
};

struct BufferRequest {
    v4l2_buffer buffer{};
    v4l2_plane plane{};

    explicit BufferRequest(v4l2_buf_type type, std::uint32_t index = 0) {
        buffer.type = type;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;
        if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            buffer.m.planes = &plane;
            buffer.length = 1;
        }
    }
};

}  // namespace

struct Capture::State {
    std::string path;
    UniqueFd fd;
    v4l2_buf_type type{V4L2_BUF_TYPE_VIDEO_CAPTURE};
    CaptureFormat format{};
    std::unique_ptr<Mapping[]> maps;
    std::size_t y_bytes{};
    std::size_t image_bytes{};
    bool bt709{};
    bool full_range{};
    bool streaming{};
    std::atomic<int> requeue_error{};

    ~State() {
        if (streaming) xioctl(fd.get(), VIDIOC_STREAMOFF, &type);
    }

    void requeue(std::uint32_t index) noexcept {
        BufferRequest request(type, index);
        if (xioctl(fd.get(), VIDIOC_QBUF, &request.buffer) < 0) requeue_error.store(errno);
    }

    struct Lease : Frame {
        std::shared_ptr<State> owner;
        std::uint32_t index;

        Lease(std::shared_ptr<State> state, std::uint32_t buffer_index)
            : owner(std::move(state)), index(buffer_index) {}
        ~Lease() { owner->requeue(index); }
    };
};

Capture::Capture(const Config& config, bool export_dma) : state_(std::make_shared<State>()) {
    auto& state = *state_;
    state.path = resolve_device(config.video_device);
    state.fd.reset(::open(state.path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC));
    const auto fd = state.fd.get();
    check(fd, "open " + state.path);
    v4l2_capability caps{};
    check(xioctl(fd, VIDIOC_QUERYCAP, &caps), "VIDIOC_QUERYCAP");
    const auto flags = caps.capabilities & V4L2_CAP_DEVICE_CAPS ? caps.device_caps : caps.capabilities;
    const bool mp = (flags & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0;
    if (!(flags & V4L2_CAP_STREAMING) || !(mp || (flags & V4L2_CAP_VIDEO_CAPTURE))) {
        throw std::runtime_error("select a V4L2 streaming capture node, such as rkisp_mainpath");
    }
    state.type = mp ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    auto& actual = state.format;
    actual.multiplanar = mp;

    v4l2_format fmt{};
    fmt.type = state.type;
    if (mp) {
        fmt.fmt.pix_mp.width = config.width;
        fmt.fmt.pix_mp.height = config.height;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
    } else {
        fmt.fmt.pix.width = config.width;
        fmt.fmt.pix.height = config.height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }
    check(xioctl(fd, VIDIOC_S_FMT, &fmt), "VIDIOC_S_FMT NV12");

    const auto fourcc = mp ? fmt.fmt.pix_mp.pixelformat : fmt.fmt.pix.pixelformat;
    const auto width = mp ? fmt.fmt.pix_mp.width : fmt.fmt.pix.width;
    const auto height = mp ? fmt.fmt.pix_mp.height : fmt.fmt.pix.height;
    const auto stride = mp ? fmt.fmt.pix_mp.plane_fmt[0].bytesperline : fmt.fmt.pix.bytesperline;
    const auto size = mp ? fmt.fmt.pix_mp.plane_fmt[0].sizeimage : fmt.fmt.pix.sizeimage;
    if (fourcc != V4L2_PIX_FMT_NV12 || (mp && fmt.fmt.pix_mp.num_planes != 1)) {
        throw std::runtime_error("driver did not return contiguous NV12; check --list-formats-ext on the ISP output");
    }
    if (width == 0 || width > 8192 || height == 0 || height > 8192 || width % 2 || height % 2 ||
        stride < width || stride % 2 || stride > static_cast<unsigned>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("invalid NV12 dimensions or bytesperline from V4L2");
    }
    actual.width = static_cast<int>(width);
    actual.height = static_cast<int>(height);
    actual.stride = static_cast<int>(stride);
    state.y_bytes = static_cast<std::size_t>(stride) * height;
    state.image_bytes = state.y_bytes + state.y_bytes / 2;
    if (state.image_bytes > size) throw std::runtime_error("V4L2 sizeimage is too small for NV12");

    const unsigned colorspace = mp ? fmt.fmt.pix_mp.colorspace : fmt.fmt.pix.colorspace;
    const unsigned encoding = mp ? fmt.fmt.pix_mp.ycbcr_enc : fmt.fmt.pix.ycbcr_enc;
    const unsigned quantization = mp ? fmt.fmt.pix_mp.quantization : fmt.fmt.pix.quantization;
    state.bt709 = encoding == V4L2_YCBCR_ENC_709 ||
                  (encoding == V4L2_YCBCR_ENC_DEFAULT && colorspace == V4L2_COLORSPACE_REC709);
    state.full_range = quantization == V4L2_QUANTIZATION_FULL_RANGE ||
                       (quantization == V4L2_QUANTIZATION_DEFAULT && colorspace == V4L2_COLORSPACE_JPEG);

    v4l2_streamparm parm{};
    parm.type = state.type;
    parm.parm.capture.timeperframe = {1, static_cast<std::uint32_t>(config.fps)};
    xioctl(fd, VIDIOC_S_PARM, &parm);
    parm = {};
    parm.type = state.type;
    actual.fps_confirmed = xioctl(fd, VIDIOC_G_PARM, &parm) == 0 &&
                           parm.parm.capture.timeperframe.denominator != 0 &&
                           parm.parm.capture.timeperframe.numerator != 0;
    actual.fps_num = actual.fps_confirmed ? parm.parm.capture.timeperframe.denominator : config.fps;
    actual.fps_den = actual.fps_confirmed ? parm.parm.capture.timeperframe.numerator : 1;

    v4l2_requestbuffers buffers{};
    buffers.type = state.type;
    buffers.memory = V4L2_MEMORY_MMAP;
    buffers.count = config.buffers;
    check(xioctl(fd, VIDIOC_REQBUFS, &buffers), "VIDIOC_REQBUFS");
    const auto held_by_inference = config.model.empty() ? 0 : config.inference_queue + 1;
    const auto held_by_stream = config.h264_file.empty() && config.rtmp_url.empty()
                                    ? 0 : config.stream_queue + 1;
    const auto minimum = static_cast<unsigned>(held_by_inference + held_by_stream + 3);
    if (buffers.count < minimum) {
        throw std::runtime_error("insufficient capture buffers for active queues; increase buffers or shrink queues");
    }
    actual.buffers = buffers.count;
    state.maps = std::make_unique<Mapping[]>(buffers.count);
    actual.dma_exported = export_dma;
    for (std::uint32_t index = 0; index < buffers.count; ++index) {
        BufferRequest request(state.type, index);
        check(xioctl(fd, VIDIOC_QUERYBUF, &request.buffer), "VIDIOC_QUERYBUF");
        auto& map = state.maps[index];
        map.length = mp ? request.plane.length : request.buffer.length;
        if (map.length < state.image_bytes) throw std::runtime_error("V4L2 buffer is too small for NV12");
        const auto offset = mp ? request.plane.m.mem_offset : request.buffer.m.offset;
        map.address = ::mmap(nullptr, map.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
        if (map.address == MAP_FAILED) check(-1, "mmap V4L2 buffer");

        if (actual.dma_exported) {
            v4l2_exportbuffer exported{};
            exported.type = state.type;
            exported.index = index;
            exported.flags = O_CLOEXEC | O_RDWR;
            if (xioctl(fd, VIDIOC_EXPBUF, &exported) == 0) {
                map.dma_fd.reset(exported.fd);
            } else {
                std::cerr << "VIDIOC_EXPBUF unavailable; using mapped frames: " << std::strerror(errno) << '\n';
                actual.dma_exported = false;
                for (std::uint32_t previous = 0; previous < index; ++previous) {
                    state.maps[previous].dma_fd.reset();
                }
            }
        }
    }
}
// Capture对象的析构函数，负责释放资源，包括关闭文件描述符和取消内存映射。
const CaptureFormat& Capture::format() const { return state_->format; }
const std::string& Capture::device() const { return state_->path; }

void Capture::start() {
    auto& state = *state_;
    for (std::uint32_t index = 0; index < state.format.buffers; ++index) {
        BufferRequest request(state.type, index);
        check(xioctl(state.fd.get(), VIDIOC_QBUF, &request.buffer), "VIDIOC_QBUF");
    }
    check(xioctl(state.fd.get(), VIDIOC_STREAMON, &state.type), "VIDIOC_STREAMON");
    state.streaming = true;
}

FramePtr Capture::next(int timeout_ms) {
    auto& state = *state_;
    const auto previous_error = state.requeue_error.load();
    if (previous_error != 0) {
        throw std::system_error(previous_error, std::generic_category(), "VIDIOC_QBUF after capture");
    }
    pollfd watched{state.fd.get(), POLLIN, 0};
    const int ready = ::poll(&watched, 1, timeout_ms);
    if (ready == 0 || (ready < 0 && errno == EINTR)) return {};
    check(ready, "poll V4L2");
    if (!(watched.revents & POLLIN)) throw std::runtime_error("V4L2 poll returned a capture error");

    BufferRequest request(state.type);
    const int dequeued = xioctl(state.fd.get(), VIDIOC_DQBUF, &request.buffer);
    if (dequeued < 0 && errno == EAGAIN) return {};
    check(dequeued, "VIDIOC_DQBUF");
    const auto index = request.buffer.index;
    if (index >= state.format.buffers) throw std::runtime_error("invalid V4L2 buffer index");
    const auto& map = state.maps[index];
    const auto offset = state.format.multiplanar ? request.plane.data_offset : 0U;
    const auto used = state.format.multiplanar ? request.plane.bytesused : request.buffer.bytesused;
    if (request.buffer.flags & V4L2_BUF_FLAG_ERROR) {
        state.requeue(index);
        return {};
    }
    if (used > map.length || offset > used || used - offset < state.image_bytes) {
        state.requeue(index);
        throw std::runtime_error("short or invalid NV12 payload returned by V4L2");
    }
    std::shared_ptr<State::Lease> frame;
    try {
        frame = std::make_shared<State::Lease>(state_, index);
    } catch (...) {
        state.requeue(index);
        throw;
    }
    const auto* address = static_cast<const std::uint8_t*>(map.address) + offset;
    frame->image = {address, address + state.y_bytes, state.format.width, state.format.height,
                    state.format.stride, state.bt709, state.full_range};
    frame->sequence = request.buffer.sequence;
    // RGA and MPP import from the beginning of the DMA-BUF.
    frame->dma_fd = offset == 0 ? map.dma_fd.get() : -1;
    frame->dma_bytes = map.length;
    return frame;
}

void save_nv12(const Nv12View& image, const std::string& path) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) throw std::runtime_error("cannot open snapshot: " + path);
    const auto write_plane = [&](const std::uint8_t* plane, int height) {
        for (int row = 0; row < height; ++row) {
            output.write(reinterpret_cast<const char*>(plane + static_cast<std::size_t>(row) * image.stride),
                         image.width);
        }
    };
    write_plane(image.y, image.height);
    write_plane(image.uv, image.height / 2);
    output.close();
    if (!output) throw std::runtime_error("cannot write snapshot: " + path);
}

}  // namespace robot
