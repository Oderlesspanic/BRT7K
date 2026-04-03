#pragma once
#include <cstdint>
#include <string>
#include <vector>

class I2CBus {
public:
    I2CBus(const std::string& device, int address);
    ~I2CBus();

    bool open_bus();
    void close_bus();

    bool write_byte(uint8_t reg, uint8_t value);
    bool read_byte(uint8_t reg, uint8_t& value);
    bool read_bytes(uint8_t reg, uint8_t* buffer, size_t length);

private:
    std::string device_;
    int address_;
    int fd_;
};