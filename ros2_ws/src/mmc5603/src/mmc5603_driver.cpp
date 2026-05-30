#include "mmc5603/mmc5603_driver.hpp"
#include <unistd.h>

namespace {
constexpr uint8_t REG_XOUT0 = 0x00;
constexpr uint8_t REG_STATUS1 = 0x18;
constexpr uint8_t REG_INTERNAL_CONTROL_0 = 0x1B;
constexpr uint8_t REG_INTERNAL_CONTROL_1 = 0x1C;

constexpr uint8_t CTRL0_TAKE_MEAS_M = 0x01;
constexpr uint8_t CTRL1_SOFTWARE_RESET = 0x80;
constexpr uint8_t STATUS_MEAS_M_DONE = 0x40;
}

MMC5603Driver::MMC5603Driver(std::shared_ptr<I2CBus> bus)
: bus_(bus) {}

bool MMC5603Driver::initialize() {
    if (!bus_->write_byte(REG_INTERNAL_CONTROL_1, CTRL1_SOFTWARE_RESET)) {
        return false;
    }

    usleep(20000);
    return true;
}

bool MMC5603Driver::trigger_measurement() {
    return bus_->write_byte(REG_INTERNAL_CONTROL_0, CTRL0_TAKE_MEAS_M);
}

bool MMC5603Driver::wait_for_measurement(std::chrono::milliseconds timeout) {
    const auto sleep_step = std::chrono::milliseconds(1);
    auto waited = std::chrono::milliseconds(0);

    while (waited <= timeout) {
        uint8_t status = 0;

        if (!bus_->read_byte(REG_STATUS1, status)) {
            return false;
        }

        if ((status & STATUS_MEAS_M_DONE) != 0) {
            return true;
        }

        usleep(static_cast<useconds_t>(sleep_step.count() * 1000));
        waited += sleep_step;
    }

    return false;
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
