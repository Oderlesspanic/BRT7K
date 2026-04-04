#include "camera/image_converter.hpp"

#include <algorithm>
#include <cstddef>

namespace
{
inline uint8_t clamp_to_u8(int value)
{
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}
}

void ImageConverter::yuv420_to_rgb(
    const uint8_t* y_plane,
    const uint8_t* u_plane,
    const uint8_t* v_plane,
    int width,
    int height,
    int y_stride,
    int uv_stride,
    std::vector<uint8_t>& rgb_out)
{
    rgb_out.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 3);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int Y = y_plane[y * y_stride + x];
            const int U = u_plane[(y / 2) * uv_stride + (x / 2)];
            const int V = v_plane[(y / 2) * uv_stride + (x / 2)];

            const int C = Y - 16;
            const int D = U - 128;
            const int E = V - 128;

            const int R = (298 * C + 409 * E + 128) >> 8;
            const int G = (298 * C - 100 * D - 208 * E + 128) >> 8;
            const int B = (298 * C + 516 * D + 128) >> 8;

            const size_t idx = static_cast<size_t>(y * width + x) * 3;
            rgb_out[idx + 0] = clamp_to_u8(R);
            rgb_out[idx + 1] = clamp_to_u8(G);
            rgb_out[idx + 2] = clamp_to_u8(B);
        }
    }
}