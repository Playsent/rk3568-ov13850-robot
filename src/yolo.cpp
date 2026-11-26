#include "robot/yolo.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot {
namespace {

constexpr int anchors[3][3][2] = {
    {{10, 13}, {16, 30}, {33, 23}},
    {{30, 61}, {62, 45}, {59, 119}},
    {{116, 90}, {156, 198}, {373, 326}},
};

float overlap(const Box& a, const Box& b) {
    const auto w = std::max(0.0F, std::min(a.right, b.right) - std::max(a.left, b.left));
    const auto h = std::max(0.0F, std::min(a.bottom, b.bottom) - std::max(a.top, b.top));
    const auto area = [](const Box& box) {
        return (box.right - box.left) * (box.bottom - box.top);
    };
    return w * h / (area(a) + area(b) - w * h);
}

}  // namespace

std::vector<Box> decode_yolo(const std::array<YoloHead, 3>& heads, const Letterbox& geometry,
                             int classes, float score_threshold, float nms_threshold) {
    const int fields = classes + 5;
    std::array<bool, 3> seen{};
    std::vector<Box> candidates;
    for (const auto& head : heads) {
        if (head.grid_width <= 0 || head.grid_height <= 0 || !head.data) {
            throw std::runtime_error("empty YOLOv5 output tensor");
        }
        const int stride = geometry.input_width / head.grid_width;
        int anchor_level;
        switch (stride) {
            case 8: anchor_level = 0; break;
            case 16: anchor_level = 1; break;
            case 32: anchor_level = 2; break;
            default: throw std::runtime_error("expected YOLOv5 heads with strides 8, 16 and 32");
        }
        const auto cells = static_cast<std::size_t>(head.grid_width) * head.grid_height;
        if (seen[anchor_level] || geometry.input_width != head.grid_width * stride ||
            geometry.input_height != head.grid_height * stride ||
            head.elements != cells * fields * 3) {
            throw std::runtime_error("invalid YOLOv5 output shape or labels");
        }
        seen[anchor_level] = true;

        const auto to_source_x = [&](float x) {
            const auto scaled = (x - geometry.x) * geometry.source_width / geometry.width;
            return std::clamp(scaled, 0.0F, static_cast<float>(geometry.source_width));
        };
        const auto to_source_y = [&](float y) {
            const auto scaled = (y - geometry.y) * geometry.source_height / geometry.height;
            return std::clamp(scaled, 0.0F, static_cast<float>(geometry.source_height));
        };

        for (int anchor = 0; anchor < 3; ++anchor) {
            for (std::size_t cell = 0; cell < cells; ++cell) {
                const auto value = [&](int field) {
                    const auto channel = static_cast<std::size_t>(anchor * fields + field);
                    return head.channels_last ? head.data[cell * fields * 3 + channel]
                                              : head.data[channel * cells + cell];
                };
                // Rockchip Model Zoo exports the three heads after sigmoid.
                const auto objectness = value(4);
                if (!std::isfinite(objectness) || objectness < score_threshold) continue;
                int class_id = 0;
                for (int label = 1; label < classes; ++label) {
                    if (value(5 + label) > value(5 + class_id)) class_id = label;
                }
                const auto score = objectness * value(5 + class_id);
                if (!std::isfinite(score) || score < score_threshold || score > 1.02F) continue;
                const auto x = (value(0) * 2 - 0.5F + static_cast<float>(cell % head.grid_width)) * stride;
                const auto y = (value(1) * 2 - 0.5F + static_cast<float>(cell / head.grid_width)) * stride;
                const auto w = 4 * value(2) * value(2) * anchors[anchor_level][anchor][0];
                const auto h = 4 * value(3) * value(3) * anchors[anchor_level][anchor][1];
                Box box{class_id, std::min(score, 1.0F), to_source_x(x - w / 2), to_source_y(y - h / 2),
                        to_source_x(x + w / 2), to_source_y(y + h / 2)};
                if (box.left < box.right && box.top < box.bottom) candidates.push_back(box);
            }
        }
    }

    std::stable_sort(candidates.begin(), candidates.end(), [](const Box& a, const Box& b) {
        return a.score > b.score;
    });
    std::vector<Box> kept;
    for (const auto& box : candidates) {
        const auto suppressed = std::any_of(kept.begin(), kept.end(), [&](const Box& selected) {
            return selected.class_id == box.class_id && overlap(selected, box) > nms_threshold;
        });
        if (!suppressed) kept.push_back(box);
        if (kept.size() == 128) break;
    }
    return kept;
}

}  // namespace robot
