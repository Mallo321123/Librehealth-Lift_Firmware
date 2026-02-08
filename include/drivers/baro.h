#ifndef BARO_H
#define BARO_H

#include "MS5611.h"
#include "Arduino.h"
#include "kalman.h"
#include "hardware-reference.h"

namespace baro
{

    struct BaroConfig
    {
        osr_t osr;                 // Oversampling Rate
        int8_t read_delay_ms;      // Delay after triggering a read, in milliseconds
        bool kalman_filter = true; // Whether to apply Kalman filtering to the pressure readings

        constexpr BaroConfig(osr_t osr_ = OSR_ULTRA_HIGH, int8_t read_delay_ms_ = 1, bool kalman_filter_ = true) noexcept
            : osr(osr_), read_delay_ms(read_delay_ms_), kalman_filter(kalman_filter_) {}
    };

    enum class State
    {
        Uninitialized,
        Initialized,
        Ready,
        Running
    };

    struct MeasureTimeItem
    {
        osr_t osr;
        float time; // in millis
    };

    inline const MeasureTimeItem measureTimeMap[] = {
        {OSR_ULTRA_HIGH, 8.22f},
        {OSR_HIGH, 4.11f},
        {OSR_STANDARD, 2.1f},
        {OSR_LOW, 1.1f},
        {OSR_ULTRA_LOW, 0.5f}};
}

class Barometer
{
    MS5611 ms5611;
    baro::State state = baro::State::Uninitialized;
    baro::BaroConfig config = baro::BaroConfig{};

    float temperature = NAN;
    float pressure = NAN;

    KalmanFilter kalmanPressure{KalmanFilter::pressure};

    SemaphoreHandle_t dataMutex = nullptr;
    TaskHandle_t valueTaskHandle = nullptr;

    static void valueUpdateTask(void *pvParameters);

public:
    bool debug = false;

    Barometer();

    bool begin(hardware::baro baroConfig = hardware::BARO_DEFAULT);
    bool suspend();
    bool resume(hardware::baro baroConfig = hardware::BARO_DEFAULT);
    bool start();
    bool stop();

    float getPressure();
};

#endif // BARO_H