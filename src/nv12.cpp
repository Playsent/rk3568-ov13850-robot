#include "robot/nv12.hpp"

#include <algorithm>
#include <stdexcept>

namespace robot {

Letterbox make_letterbox(int width, int height, int input_width, int input_height) {
    if (width <= 0 || height <= 0 || input_width < 4 || input_height < 4 ||
        input_width % 4 || input_height % 4) {
        throw std::invalid_argument("invalid letterbox dimensions");
    }
    const auto scale = std::min(static_cast<double>(input_width) / width,
                                static_cast<double>(input_height) / height);
    // Even offsets and dimensions also meet the RGA rectangle alignment.
    const int scaled_width = std::max(4, static_cast<int>(width * scale + 1e-6) & ~3);
    const int scaled_height = std::max(4, static_cast<int>(height * scale + 1e-6) & ~3);
    return {input_width, input_height, width, height,
            (input_width - scaled_width) / 2, (input_height - scaled_height) / 2,
            scaled_width, scaled_height};
}

void nv12_to_rgb(const Nv12View& image, const Letterbox& geometry,
                 std::vector<std::uint8_t>& output) {
    output.assign(static_cast<std::size_t>(geometry.input_width) * geometry.input_height * 3, 114);
    const auto channel = [](int value) {
        return static_cast<std::uint8_t>(std::clamp((value + 128) >> 8, 0, 255));
    };
    const int y_scale = image.full_range ? 256 : 298;
    const int y_black = image.full_range ? 0 : 16;
    const int r_v = image.bt709 ? (image.full_range ? 403 : 459) : (image.full_range ? 359 : 409);
    const int g_u = image.bt709 ? (image.full_range ? 48 : 55) : (image.full_range ? 88 : 100);
    const int g_v = image.bt709 ? (image.full_range ? 120 : 136) : (image.full_range ? 183 : 208);
    const int b_u = image.bt709 ? (image.full_range ? 475 : 541) : (image.full_range ? 454 : 516);
    for (int y = 0; y < geometry.height; ++y) {
        const int src_y = (2 * y + 1) * image.height / (2 * geometry.height);
        const auto* y_row = image.y + static_cast<std::size_t>(src_y) * image.stride;
        const auto* uv_row = image.uv + static_cast<std::size_t>(src_y / 2) * image.stride;
        auto* rgb = output.data() +
                    (static_cast<std::size_t>(y + geometry.y) * geometry.input_width + geometry.x) * 3;
        for (int x = 0; x < geometry.width; ++x) {
            const int src_x = (2 * x + 1) * image.width / (2 * geometry.width);
            const int luma = (y_row[src_x] - y_black) * y_scale;
            const int u = uv_row[src_x & ~1] - 128;
            const int v = uv_row[(src_x & ~1) + 1] - 128;
            *rgb++ = channel(luma + r_v * v);
            *rgb++ = channel(luma - g_u * u - g_v * v);
            *rgb++ = channel(luma + b_u * u);
        }
    }
}

}  // namespace robot
