#include "mmc5603/mmc5603_driver.hpp"
#include <unistd.h>

#define REG_XOUT0 0x00

MMC5603Driver::MMC5603Driver(std::shared_ptr<I2CBus> bus)
: bus_(bus) {}

bool MMC5603Driver::initialize() {
    bus_->write_byte(0x1C, 0x80);
    usleep(10000);
    return true;
}

bool MMC5603Driver::trigger_measurement() {
    // Start measurement
    return bus_->write_byte(0x1B, 0x01);
}

int32_t MMC5603Driver::combine_20bit(uint8_t msb, uint8_t mid, uint8_t lsb) {
    return (msb << 12) | (mid << 4) | ((lsb >> 4) & 0x0F);
}

bool MMC5603Driver::read_raw_data(MMC5603RawData& data) {

    uint8_t buffer[9];

    if (!bus_->read_bytes(REG_XOUT0, buffer, 9)) {
        return false;
    }

    data.x = combine_20bit(buffer[0], buffer[1], buffer[6]);
    data.y = combine_20bit(buffer[2], buffer[3], buffer[7]);
    data.z = combine_20bit(buffer[4], buffer[5], buffer[8]);

    return true;
}