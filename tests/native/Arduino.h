#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
struct TestSerial {
    std::string output;
    size_t write(uint8_t c) { output += static_cast<char>(c); return 1; }
    size_t write(const uint8_t* data, size_t n) {
        output.append(reinterpret_cast<const char*>(data), n); return n;
    }
    void println() { output += '\n'; }
};
inline TestSerial Serial;
