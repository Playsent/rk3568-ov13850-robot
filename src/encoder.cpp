#include "robot/encoder.hpp"
#include "robot/rtmp.hpp"

#define MODULE_TAG "inspection_robot"
#include <rk_mpi.h>
#include <mpp_buffer.h>
#include <mpp_frame.h>
#include <mpp_packet.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace robot {
namespace {

void check(MPP_RET result, const char* operation) {
    if (result != MPP_OK) {
        throw std::runtime_error(std::string(operation) + " failed (" + std::to_string(result) + ")");
    }
}

int align16(int value) { return (value + 15) & ~15; }

struct Buffer {
    MppBuffer value{};
    ~Buffer() { if (value) mpp_buffer_put(value); }
};

struct VideoFrame {
    MppFrame value{};
    ~VideoFrame() { if (value) mpp_frame_deinit(&value); }
};

struct Packet {
    MppPacket value{};
    ~Packet() { if (value) mpp_packet_deinit(&value); }
};

bool has_idr(const std::uint8_t* data, std::size_t size) {
    for (std::size_t index = 2; index + 1 < size; ++index) {
        if (data[index - 2] == 0 && data[index - 1] == 0 && data[index] == 1 &&
            (data[index + 1] & 31) == 5) return true;
    }
    return false;
}

}  // namespace

struct Stream::Impl {
    CaptureFormat capture;
    const int stride;
    const int vertical_stride;
    const std::size_t y_size;
    const std::size_t frame_size;
    bool can_import;
    bool input_logged{};
    MppCtx context{};
    MppApi* api{};
    MppBufferGroup group{};
    Buffer staging;
    std::ofstream file;
    Rtmp rtmp;

    Impl(const Config& config, const CaptureFormat& actual)
        : capture(actual), stride(align16(actual.stride)), vertical_stride(align16(actual.height)),
          y_size(static_cast<std::size_t>(stride) * vertical_stride), frame_size(y_size * 3 / 2),
          can_import(actual.dma_exported && stride == actual.stride && vertical_stride == actual.height),
          rtmp(config, std::to_string(actual.fps_num) + "/" + std::to_string(actual.fps_den)) {}

    ~Impl() {
        if (context) mpp_destroy(context);
        if (staging.value) {
            mpp_buffer_put(staging.value);
            staging.value = nullptr;
        }
        if (group) mpp_buffer_group_put(group);
    }

    void init(const Config& config) {
        const double fps = static_cast<double>(capture.fps_num) / capture.fps_den;
        if (capture.width > 1920 || capture.height > 1080 || fps > 60 || fps < 1 ||
            stride > 2048) {
            throw std::runtime_error("MPP stream expects an ISP output at or below 1920x1080 and 1..60 fps");
        }
        check(mpp_create(&context, &api), "mpp_create");
        MppPollType input_timeout = MPP_POLL_BLOCK;
        MppPollType output_timeout = static_cast<MppPollType>(1000);
        check(api->control(context, MPP_SET_INPUT_TIMEOUT, &input_timeout), "MPP_SET_INPUT_TIMEOUT");
        check(api->control(context, MPP_SET_OUTPUT_TIMEOUT, &output_timeout), "MPP_SET_OUTPUT_TIMEOUT");
        check(mpp_init(context, MPP_CTX_ENC, MPP_VIDEO_CodingAVC), "mpp_init H.264");

        struct EncoderConfig {
            MppEncCfg value{};
            ~EncoderConfig() { if (value) mpp_enc_cfg_deinit(value); }
        } settings;
        check(mpp_enc_cfg_init(&settings.value), "mpp_enc_cfg_init");
        check(api->control(context, MPP_ENC_GET_CFG, settings.value), "MPP_ENC_GET_CFG");
        const auto set = [&](const char* key, int value) {
            check(mpp_enc_cfg_set_s32(settings.value, key, value), key);
        };
        const auto set_denominator = [&](const char* current, const char* legacy, int value) {
            if (mpp_enc_cfg_set_s32(settings.value, current, value) != MPP_OK) set(legacy, value);
        };
        set("prep:width", capture.width);
        set("prep:height", capture.height);
        set("prep:hor_stride", stride);
        set("prep:ver_stride", vertical_stride);
        set("prep:format", MPP_FMT_YUV420SP);
        set("rc:mode", MPP_ENC_RC_MODE_CBR);
        set("rc:bps_target", config.bitrate);
        set("rc:bps_max", config.bitrate * 17 / 16);
        set("rc:bps_min", config.bitrate * 15 / 16);
        const auto divisor = std::gcd(capture.fps_num, capture.fps_den);
        set("rc:fps_in_flex", 0);
        set("rc:fps_out_flex", 0);
        set("rc:fps_in_num", static_cast<int>(capture.fps_num / divisor));
        set("rc:fps_out_num", static_cast<int>(capture.fps_num / divisor));
        set_denominator("rc:fps_in_denom", "rc:fps_in_denorm", static_cast<int>(capture.fps_den / divisor));
        set_denominator("rc:fps_out_denom", "rc:fps_out_denorm", static_cast<int>(capture.fps_den / divisor));
        set("rc:gop", static_cast<int>(fps * 2));
        set("h264:profile", 100);
        set("h264:level", fps <= 30 ? 40 : 42);
        check(api->control(context, MPP_ENC_SET_CFG, settings.value), "MPP_ENC_SET_CFG");
        MppEncHeaderMode header = MPP_ENC_HEADER_MODE_EACH_IDR;
        check(api->control(context, MPP_ENC_SET_HEADER_MODE, &header), "MPP_ENC_SET_HEADER_MODE");

        if (!config.h264_file.empty()) {
            file.open(config.h264_file, std::ios::binary | std::ios::trunc);
            if (!file) throw std::runtime_error("cannot open H.264 output: " + config.h264_file);
        }
    }

    MppBuffer prepare(const Frame& frame, Buffer& imported) {
        if (can_import && frame.dma_fd >= 0 && frame.dma_bytes >= frame_size) {
            MppBufferInfo info{};
            info.type = MPP_BUFFER_TYPE_DRM;
            info.fd = frame.dma_fd;
            info.size = frame.dma_bytes;
            if (mpp_buffer_import(&imported.value, &info) == MPP_OK && imported.value) {
                if (!input_logged) std::cerr << "MPP input: V4L2 DMA-BUF import\n";
                input_logged = true;
                return imported.value;
            }
            can_import = false;
        }
        if (!group) {
            check(mpp_buffer_group_get(&group, MPP_BUFFER_TYPE_DRM, MPP_BUFFER_INTERNAL,
                                       MODULE_TAG, __func__), "MPP DMA buffer group");
            check(mpp_buffer_get(group, &staging.value, frame_size), "allocate MPP NV12 buffer");
            std::cerr << "MPP input: copy visible NV12 rows into aligned encoder buffer\n";
            input_logged = true;
        }
        auto* out = static_cast<std::uint8_t*>(mpp_buffer_get_ptr(staging.value));
        if (!out) throw std::runtime_error("cannot map MPP input buffer");
#ifdef mpp_buffer_sync_begin
        check(mpp_buffer_sync_begin(staging.value), "MPP buffer sync begin");
#endif
        const auto copy_plane = [&](const std::uint8_t* src, std::uint8_t* dst,
                                    int rows, int aligned_rows, int fill) {
            std::memset(dst, fill, static_cast<std::size_t>(aligned_rows) * stride);
            for (int row = 0; row < rows; ++row) {
                std::memcpy(dst + static_cast<std::size_t>(row) * stride,
                            src + static_cast<std::size_t>(row) * frame.image.stride, capture.width);
            }
        };
        copy_plane(frame.image.y, out, capture.height, vertical_stride, frame.image.full_range ? 0 : 16);
        copy_plane(frame.image.uv, out + y_size, capture.height / 2, vertical_stride / 2, 128);
#ifdef mpp_buffer_sync_end
        check(mpp_buffer_sync_end(staging.value), "MPP buffer sync end");
#endif
        return staging.value;
    }
};

Stream::Stream(const Config& config, const CaptureFormat& capture)
    : impl_(std::make_unique<Impl>(config, capture)) { impl_->init(config); }
Stream::~Stream() = default;

void Stream::process(FramePtr frame) {
    auto& state = *impl_;
    Buffer imported;
    const auto input = state.prepare(*frame, imported);
    VideoFrame video;
    check(mpp_frame_init(&video.value), "mpp_frame_init");
    mpp_frame_set_width(video.value, state.capture.width);
    mpp_frame_set_height(video.value, state.capture.height);
    mpp_frame_set_hor_stride(video.value, state.stride);
    mpp_frame_set_ver_stride(video.value, state.vertical_stride);
    mpp_frame_set_fmt(video.value, MPP_FMT_YUV420SP);
    mpp_frame_set_pts(video.value, frame->sequence);
    mpp_frame_set_buffer(video.value, input);
    Packet packet;
    try {
        check(state.api->encode_put_frame(state.context, video.value), "MPP encode_put_frame");
        check(state.api->encode_get_packet(state.context, &packet.value), "MPP encode_get_packet");
        if (!packet.value) throw std::runtime_error("MPP returned no H.264 packet");
    } catch (...) {
        // Stop the encoder before the V4L2 lease can requeue an imported buffer.
        mpp_destroy(state.context);
        state.context = nullptr;
        throw;
    }
    mpp_frame_deinit(&video.value);
    if (imported.value) {
        mpp_buffer_put(imported.value);
        imported.value = nullptr;
    }
    frame.reset();

    const auto size = mpp_packet_get_length(packet.value);
    const auto* data = static_cast<const std::uint8_t*>(mpp_packet_get_pos(packet.value));
    if (size == 0) return;
    if (!data) throw std::runtime_error("MPP returned a packet without data");
    if (state.file.is_open()) {
        state.file.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
        if (!state.file) throw std::runtime_error("failed writing H.264 output");
    }
    state.rtmp.publish(data, size, has_idr(data, size));
}

void Stream::finish() {
    if (impl_->file.is_open()) {
        impl_->file.close();
        if (!impl_->file) throw std::runtime_error("failed closing H.264 output");
    }
    impl_->rtmp.finish();
}

}  // namespace robot
