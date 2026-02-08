
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

        constexpr SPIBusConfig(int8_t mosi_ = 47,
                               int8_t miso_ = 45,
                               int8_t sclk_ = 48,
                               int8_t cs_ = 42) noexcept
            : mosi(mosi_), miso(miso_), sclk(sclk_), cs(cs_) {}
    };

    struct SDCardConfig
    {
        int8_t clk; // SCK Pin (GPIO)
        int8_t cmd; // CMD Pin (GPIO)
        int8_t d0;  // D0 Pin (GPIO)
        int8_t d1;  // D1 Pin (GPIO)
        int8_t d2;  // D2 Pin (GPIO)
        int8_t d3;  // D3 Pin (GPIO)

        constexpr SDCardConfig(int8_t clk_ = 14,
                                 int8_t cmd_ = 15,
                                 int8_t d0_ = 2,
                                 int8_t d1_ = 4,
                                 int8_t d2_ = 12,
                                 int8_t d3_ = 13) noexcept
            : clk(clk_), cmd(cmd_), d0(d0_), d1(d1_), d2(d2_), d3(d3_) {}
    };

    struct baro
    {
        int8_t sda; // SDA Pin (GPIO)
        int8_t scl; // SCL Pin (GPIO)
        int8_t address; // I2C Adresse

        constexpr baro(int8_t sda_ = 8,
                       int8_t scl_ = 9,
                       int8_t address_ = 0x77) noexcept
            : sda(sda_), scl(scl_), address(address_) {}
    };

    static constexpr SPIBusConfig SPI_DEFAULT{};
    static constexpr SDCardConfig SDCARD_DEFAULT{};
    static constexpr baro BARO_DEFAULT{};

} // namespace hardware

#endif // HARDWARE_REFERENCE_H