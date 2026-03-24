#pragma once

class MockI2CBus : public II2CBus {
public:
    MOCK_METHOD(bool, read_bytes, (uint8_t reg, std::vector<uint8_t>& data), (override));
};