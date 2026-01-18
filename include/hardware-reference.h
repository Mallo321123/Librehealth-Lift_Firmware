
#ifndef HARDWARE_REFERENCE_H
#define HARDWARE_REFERENCE_H

// hardware-reference.h
// Config für einen Hardware SPI-Bus (ESP32-geeignete Defaults)

#include <stdint.h>

namespace hardware
{
    struct SPIBusConfig
    {
        int8_t mosi; // MOSI Pin (GPIO)
        int8_t miso; // MISO Pin (GPIO)
        int8_t sclk; // SCLK Pin (GPIO)
        int8_t cs;   // Chip Select Pin (GPIO)

        constexpr SPIBusConfig(int8_t mosi_ = 11,
                               int8_t miso_ = 45,
                               int8_t sclk_ = 48,
                               int8_t cs_ = 5) noexcept
            : mosi(mosi_), miso(miso_), sclk(sclk_), cs(cs_) {}
    };

    static constexpr SPIBusConfig SPI_DEFAULT{};

} // namespace hardware

#endif // HARDWARE_REFERENCE_H