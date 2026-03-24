#include <gtest/gtest.h>
#include <cstdint>
#include <cmath>

#include "imu/imu_converter.hpp"

TEST(BytesToInt16Test, ConvertsZeroCorrectly)
{
    EXPECT_EQ(bytes_to_int16(0x00, 0x00), 0);
}

TEST(BytesToInt16Test, ConvertsPositiveValueCorrectly)
{
    EXPECT_EQ(bytes_to_int16(0x00, 0x01), 1);
}

TEST(BytesToInt16Test, ConvertsMaxPositiveCorrectly)
{
    EXPECT_EQ(bytes_to_int16(0x7F, 0xFF), 32767);
}

TEST(BytesToInt16Test, ConvertsMinNegativeCorrectly)
{
    EXPECT_EQ(bytes_to_int16(0x80, 0x00), -32768);
}

TEST(BytesToInt16Test, ConvertsMinusOneCorrectly)
{
    EXPECT_EQ(bytes_to_int16(0xFF, 0xFF), -1);
}

TEST(AccelRawToMs2Test, ConvertsZeroCorrectly)
{
    EXPECT_NEAR(accel_raw_to_ms2(0, 16384.0), 0.0, 1e-9);
}

TEST(AccelRawToMs2Test, ConvertsOneGCorrectly)
{
    EXPECT_NEAR(accel_raw_to_ms2(16384, 16384.0), 9.80665, 1e-6);
}

TEST(AccelRawToMs2Test, ConvertsMinusOneGCorrectly)
{
    EXPECT_NEAR(accel_raw_to_ms2(-16384, 16384.0), -9.80665, 1e-6);
}

TEST(AccelRawToMs2Test, ConvertsHalfGCorrectly)
{
    EXPECT_NEAR(accel_raw_to_ms2(8192, 16384.0), 4.903325, 1e-6);
}

TEST(GyroRawToRadsTest, ConvertsZeroCorrectly)
{
    EXPECT_NEAR(gyro_raw_to_rads(0, 131.0), 0.0, 1e-9);
}

TEST(GyroRawToRadsTest, ConvertsOneDegreePerSecondCorrectly)
{
    EXPECT_NEAR(gyro_raw_to_rads(131, 131.0), M_PI / 180.0, 1e-9);
}

TEST(GyroRawToRadsTest, ConvertsMinusOneDegreePerSecondCorrectly)
{
    EXPECT_NEAR(gyro_raw_to_rads(-131, 131.0), -M_PI / 180.0, 1e-9);
}

TEST(GyroRawToRadsTest, ConvertsTwoDegreesPerSecondCorrectly)
{
    EXPECT_NEAR(gyro_raw_to_rads(262, 131.0), 2.0 * M_PI / 180.0, 1e-9);
}

TEST(TempRawToCelsiusTest, ConvertsZeroCorrectly)
{
    EXPECT_NEAR(temp_raw_to_celsius(0), 36.53, 1e-9);
}

TEST(TempRawToCelsiusTest, ConvertsPositiveRawCorrectly)
{
    EXPECT_NEAR(temp_raw_to_celsius(340), 37.53, 1e-9);
}

TEST(TempRawToCelsiusTest, ConvertsNegativeRawCorrectly)
{
    EXPECT_NEAR(temp_raw_to_celsius(-340), 35.53, 1e-9);
}

