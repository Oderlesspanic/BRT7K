#pragma once

#include <cstdint>
#include <string>
#include <vector>

class I2CBus
{
public:
    I2CBus(const std::string& device_path, int device_address);
    ~I2CBus();

    bool openBus();
    void closeBus();

    bool writeByte(uint8_t reg, uint8_t value);
    bool readByte(uint8_t reg, uint8_t& value);
    bool readBytes(uint8_t start_reg, uint8_t* buffer, std::size_t length);

    bool isOpen() const;

private:
    std::string device_path_;
    int device_address_;
    int file_descriptor_;
};