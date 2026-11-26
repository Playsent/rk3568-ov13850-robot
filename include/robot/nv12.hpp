#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robot {

struct Nv12View {
    const std::uint8_t* y{};
    const std::uint8_t* uv{};
    int width{};
    int height{};
    int stride{};
    bool bt709{};
    bool full_range{};
};

struct Letterbox {
    int input_width{};
    int input_height{};
    int source_width{};
    int source_height{};
    int x{};
    int y{};
    int width{};
    int height{};
};

Letterbox make_letterbox(int width, int height, int input_width, int input_height);
void nv12_to_rgb(const Nv12View& image, const Letterbox& geometry,
                 std::vector<std::uint8_t>& output);

}  // namespace robot
