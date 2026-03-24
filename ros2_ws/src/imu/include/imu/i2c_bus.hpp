#pragma once

class II2CBus {
public:
    virtual ~II2CBus() = default;
    virtual bool read_bytes(uint8_t reg, std::vector<uint8_t>& data) = 0;
};