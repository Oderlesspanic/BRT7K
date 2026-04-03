#pragma once
#include "mmc5603/mmc5603_driver.hpp"

struct MagneticFieldData {
    double x_tesla;
    double y_tesla;
    double z_tesla;
};

class MMC5603Convert {
public:
    static MagneticFieldData raw_to_tesla(
        const MMC5603RawData& raw,
        double offset_x,
        double offset_y,
        double offset_z,
        double scale_x,
        double scale_y,
        double scale_z
    );
};