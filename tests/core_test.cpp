#include "robot/bounded_queue.hpp"
#include "robot/config.hpp"
#include "robot/nv12.hpp"
#include "robot/yolo.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void expect(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void check_stride_and_padding() {
    constexpr int width = 8;
    constexpr int height = 4;
    constexpr int stride = 12;
    std::array<std::uint8_t, stride * height> luma{};
    std::array<std::uint8_t, stride * height / 2> chroma{};
    for (int row = 0; row < height; ++row) {
        std::fill_n(luma.data() + row * stride, width, row == 0 ? 16 : 235);
    }
    for (int row = 0; row < height / 2; ++row) {
        std::fill_n(chroma.data() + row * stride, width, 128);
    }
    const auto geometry = robot::make_letterbox(width, height, 8, 8);
    expect(geometry.x == 0 && geometry.y == 2, "letterbox placement");
    std::vector<std::uint8_t> rgb;
    robot::nv12_to_rgb({luma.data(), chroma.data(), width, height, stride, false, false},
                      geometry, rgb);
    expect(rgb.size() == 8 * 8 * 3, "RGB size");
    expect(rgb[0] == 114, "top border padding");
    expect(rgb[2 * 8 * 3] == 0, "limited range black");
    expect(rgb[5 * 8 * 3 + 23] == 255, "last pixel uses negotiated stride");
    expect(rgb[6 * 8 * 3] == 114, "bottom border padding");
}

void check_yolo_heads() {
    std::vector<float> narrow(20 * 20 * 18);
    std::vector<float> medium(40 * 40 * 18);
    std::vector<float> wide(80 * 80 * 18);
    const auto cell = 30 * 80 + 40;
    const std::array<float, 6> anchor0{0.5F, 0.5F, 0.5F, 0.5F, 0.9F, 0.9F};
    const std::array<float, 6> overlapping{
        0.5F, 0.5F, 0.5F * std::sqrt(10.F / 16.F), 0.5F * std::sqrt(13.F / 30.F), 0.8F, 0.8F};
    for (std::size_t field = 0; field < anchor0.size(); ++field) {
        wide[field * 80 * 80 + cell] = anchor0[field];
        wide[(field + 6) * 80 * 80 + cell] = overlapping[field];
    }
    const auto geometry = robot::make_letterbox(1280, 720, 640, 640);
    std::array<robot::YoloHead, 3> heads{{
        {narrow.data(), narrow.size(), 20, 20, false},
        {wide.data(), wide.size(), 80, 80, false},
        {medium.data(), medium.size(), 40, 40, true},
    }};
    const auto boxes = robot::decode_yolo(heads, geometry, 1, 0.25F, 0.45F);
    expect(boxes.size() == 1, "NMS removes the overlapping candidate");
    expect(std::abs(boxes.front().score - 0.81F) < 0.001F, "objectness times class score");
    expect(std::abs(boxes.front().left - 638.0F) < 0.01F, "x maps to sensor output");
    expect(std::abs(boxes.front().top - 195.0F) < 0.01F, "y removes letterbox padding");
    heads[1].elements -= 1;
    bool rejected = false;
    try {
        robot::decode_yolo(heads, geometry, 1, 0.25F, 0.45F);
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    expect(rejected, "short output is rejected before indexing");
}

void check_queue() {
    robot::BoundedQueue<int> queue(2);
    queue.push_latest(1);
    queue.push_latest(2);
    queue.push_latest(3);
    queue.close();
    int value = 0;
    expect(queue.dropped() == 1, "queue drop counter");
    expect(queue.pop(value) && value == 2, "oldest item is evicted");
    expect(queue.pop(value) && value == 3, "close drains the newest item");
    expect(!queue.pop(value) && !queue.push_latest(4), "closed queue is exhausted");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        expect(argc == 2, "pass the sample config path");
        const auto config = robot::load_config(argv[1]);
        expect(config.video_device == "auto" && config.width % 2 == 0 && config.height % 2 == 0,
               "default camera config");
        expect(config.model.empty() && config.rtmp_url.empty(), "default bring-up uses capture only");
        check_stride_and_padding();
        check_yolo_heads();
        check_queue();
        std::cout << "PASS config, NV12 stride and letterbox, YOLOv5 decode/NMS, bounded queue\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
