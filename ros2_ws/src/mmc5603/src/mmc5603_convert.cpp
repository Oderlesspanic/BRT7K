#include "mmc5603/mmc5603_convert.hpp"

MagneticFieldData MMC5603Convert::raw_to_tesla(
    const MMC5603RawData& raw,
    double offset_x,
    double offset_y,
    double offset_z,
    double scale_x,
    double scale_y,
    double scale_z)
{
    MagneticFieldData out{};

    constexpr double RAW_TO_UTESLA = 3000.0 / 524288.0;

    double x_ut = (static_cast<double>(raw.x) - offset_x) * RAW_TO_UTESLA * scale_x;
    double y_ut = (static_cast<double>(raw.y) - offset_y) * RAW_TO_UTESLA * scale_y;
    double z_ut = (static_cast<double>(raw.z) - offset_z) * RAW_TO_UTESLA * scale_z;

    out.x_tesla = x_ut * 1e-6;
    out.y_tesla = y_ut * 1e-6;
    out.z_tesla = z_ut * 1e-6;

    return out;
}