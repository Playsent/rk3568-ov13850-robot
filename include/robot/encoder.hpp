#pragma once

#include "robot/capture.hpp"

#include <memory>

namespace robot {

class Stream {
public:
    Stream(const Config& config, const CaptureFormat& capture);
    ~Stream();
    void process(FramePtr frame);
    void finish();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace robot
