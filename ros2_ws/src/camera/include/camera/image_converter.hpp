#pragma once

#include <cstdint>
#include <vector>

class ImageConverter
{
public:
    static void yuv420_to_bgr(
        const uint8_t* y_plane,
        const uint8_t* u_plane,
        const uint8_t* v_plane,
        int width,
        int height,
        int y_stride,
        int uv_stride,
        std::vector<uint8_t>& rgb_out);
};
