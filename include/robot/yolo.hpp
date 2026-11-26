#pragma once

#include "robot/nv12.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace robot {

struct YoloHead {
    const float* data{};
    std::size_t elements{};
    int grid_width{};
    int grid_height{};
    bool channels_last{};
};

struct Box {
    int class_id{};
    float score{};
    float left{};
    float top{};
    float right{};
    float bottom{};
};

std::vector<Box> decode_yolo(const std::array<YoloHead, 3>& heads, const Letterbox& geometry,
                             int classes, float score_threshold, float nms_threshold);

}  // namespace robot
