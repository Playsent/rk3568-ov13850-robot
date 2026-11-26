#pragma once

#include "robot/config.hpp"
#include "robot/nv12.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace robot {

struct CaptureFormat {
    int width{};
    int height{};
    int stride{};
    std::uint32_t fps_num{};
    std::uint32_t fps_den{};
    std::uint32_t buffers{};
    bool multiplanar{};
    bool dma_exported{};
    bool fps_confirmed{};
};

struct Frame {
    Nv12View image;
    std::uint32_t sequence{};
    int dma_fd{-1};
    std::size_t dma_bytes{};
};

using FramePtr = std::shared_ptr<const Frame>;

class Capture {
public:
    Capture(const Config& config, bool export_dma);
    const CaptureFormat& format() const;
    const std::string& device() const;
    void start();
    FramePtr next(int timeout_ms);

private:
    struct State;
    std::shared_ptr<State> state_;
};

void save_nv12(const Nv12View& image, const std::string& path);

}  // namespace robot
