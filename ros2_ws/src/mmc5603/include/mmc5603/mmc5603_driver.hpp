#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <chrono>
#include "mmc5603/i2c_bus.hpp"

struct MMC5603RawData {
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t temperature;
};

class MMC5603Driver {
public:
    explicit MMC5603Driver(std::shared_ptr<I2CBus> bus);

    bool initialize();
    bool trigger_measurement();
    bool wait_for_measurement(std::chrono::milliseconds timeout);
    bool read_raw_data(MMC5603RawData& data);

private:
    std::shared_ptr<I2CBus> bus_;

    int32_t combine_20bit(uint8_t msb, uint8_t mid, uint8_t low_nibble);
};
