#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "imu/mpu6050_driver.hpp"
#include "mock_i2c_bus.hpp"


TEST(MPU6050DriverTest, ReadSuccessReturnsData) {
    auto mock_bus = std::make_shared<MockI2CBus>();
    MPU6050Driver driver(mock_bus);

    std::vector<uint8_t> fake = {0x01, 0x02};

    EXPECT_CALL(*mock_bus, read_bytes(0x3B, testing::_))
        .WillOnce([&](uint8_t, std::vector<uint8_t>& out) {
            out = fake;
            return true;
        });

    auto result = driver.read_imu();

    EXPECT_TRUE(result.has_value()); 
}