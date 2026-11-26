#pragma once

#include "robot/capture.hpp"

#include <memory>

namespace robot {

class Detector {
public:
    explicit Detector(const Config& config);
    ~Detector();
    void process(FramePtr frame);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace robot
