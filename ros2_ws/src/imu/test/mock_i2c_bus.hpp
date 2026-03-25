#pragma once


class MockI2CBus : public II2CBus {
public:
    bool write(uint8_t reg, uint8_t data) override {
        return true;
    }

    bool read(uint8_t reg, uint8_t* buffer, size_t length) override {
        buffer[0] = 0x42; // Dummywert
        return true;
    }
};