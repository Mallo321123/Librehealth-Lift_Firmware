#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <Adafruit_ISM330DHCX.h>
#include "kalman.h"
#include "hardware-reference.h"

namespace imu
{
    struct IMUConfig
    {
        lsm6ds_accel_range_t la_fs;  // Linear Acceleration Full Scale
        lsm6ds_gyro_range_t gyro_fs; // Gyroscope Full Scale

        lsm6ds_data_rate_t la_odr;
        lsm6ds_data_rate_t gyro_odr;

        float autorange_acc_overhead = 1.2f;  // Multiplier for autorange calibration
        float autorange_gyro_overhead = 1.2f; // Multiplier for autorange calibration

        bool accel = true;
        bool gyro = true;

        bool accelKalman = true;
        bool gyroKalman = true;

        int32_t range_calibration_update_interval = 100; // in microseconds

        constexpr IMUConfig(lsm6ds_accel_range_t la_fs_ = LSM6DS_ACCEL_RANGE_8_G,
                            lsm6ds_gyro_range_t gyro_fs_ = LSM6DS_GYRO_RANGE_2000_DPS,
                            lsm6ds_data_rate_t la_odr_ = LSM6DS_RATE_833_HZ,
                            lsm6ds_data_rate_t gyro_odr_ = LSM6DS_RATE_416_HZ,
                            bool accel_ = true,
                            bool gyro_ = true,
                            bool accelKalman_ = true,
                            bool gyroKalman_ = true) noexcept
            : la_fs(la_fs_), gyro_fs(gyro_fs_), la_odr(la_odr_), gyro_odr(gyro_odr_),
              accel(accel_), gyro(gyro_), accelKalman(accelKalman_), gyroKalman(gyroKalman_) {}

        bool operator==(const IMUConfig &other) const
        {
            return (la_fs == other.la_fs) &&
                   (gyro_fs == other.gyro_fs) &&
                   (la_odr == other.la_odr) &&
                   (gyro_odr == other.gyro_odr) &&
                   (accel == other.accel) &&
                   (gyro == other.gyro) &&
                   (accelKalman == other.accelKalman) &&
                   (gyroKalman == other.gyroKalman);
        }

        bool operator!=(const IMUConfig &other) const
        {
            return !(*this == other);
        }
    };

    enum class State
    {
        Uninitialized,
        Initialized,
        Ready,
        Running
    };

    struct IMUReport
    {
        double accX;
        double accY;
        double accZ;

        double gyroX;
        double gyroY;
        double gyroZ;

        IMUReport operator-(const IMUReport &other) const
        {
            return IMUReport{
                accX - other.accX,
                accY - other.accY,
                accZ - other.accZ,
            };
        }
    };

    struct DatarateMapItem
    {
        lsm6ds_data_rate_t datarate;
        float frequency;
    };

    struct AccelRangeMapItem
    {
        lsm6ds_accel_range_t range;
        int max_g;
    };

    inline DatarateMapItem frequency_map[] = {
        {LSM6DS_RATE_SHUTDOWN, 0.0f},
        {LSM6DS_RATE_12_5_HZ, 12.5f},
        {LSM6DS_RATE_26_HZ, 26.0f},
        {LSM6DS_RATE_52_HZ, 52.0f},
        {LSM6DS_RATE_104_HZ, 104.0f},
        {LSM6DS_RATE_208_HZ, 208.0f},
        {LSM6DS_RATE_416_HZ, 416.0f},
        {LSM6DS_RATE_833_HZ, 833.0f},
        {LSM6DS_RATE_1_66K_HZ, 1666.0f},
        {LSM6DS_RATE_3_33K_HZ, 3332.0f},
        {LSM6DS_RATE_6_66K_HZ, 6667.0f},
    };

    inline AccelRangeMapItem accel_range_map[] = {
        {LSM6DS_ACCEL_RANGE_2_G, 2},
        {LSM6DS_ACCEL_RANGE_4_G, 4},
        {LSM6DS_ACCEL_RANGE_8_G, 8},
        {LSM6DS_ACCEL_RANGE_16_G, 16},
    };

    static constexpr IMUConfig IMU_DEFAULT{};

}

class IMUSensor
{
    Adafruit_ISM330DHCX ism330dhcx;
    imu::State state = imu::State::Uninitialized;
    imu::IMUConfig config = imu::IMUConfig{};

    imu::IMUReport report{};
    double gravityOffsetX = 0.0, gravityOffsetY = 0.0, gravityOffsetZ = 0.0;
    bool calibrated = false;

    KalmanFilter kalmanAccX{KalmanFilter::accel};
    KalmanFilter kalmanAccY{KalmanFilter::accel};
    KalmanFilter kalmanAccZ{KalmanFilter::accel};

    KalmanFilter kalmanGyroX{KalmanFilter::gyro};
    KalmanFilter kalmanGyroY{KalmanFilter::gyro};
    KalmanFilter kalmanGyroZ{KalmanFilter::gyro};

    SemaphoreHandle_t reportMutex = nullptr;
    SemaphoreHandle_t configMutex = nullptr;

    TaskHandle_t valueTaskHandle = nullptr;
    TaskHandle_t autorangeTaskHandle = nullptr;

    uint update_intervall = 10; // in milliseconds

    void applyConfig();
    static void valueUpdateTask(void *pvParameters);
    static void autorangeCalibrationTask(void *pvParameters);
    double mmToG(double mm_s2);
    void calibrateGravity(int samples = 100);

public:
    bool debug = false;

    IMUSensor();
    bool begin(hardware::SPIBusConfig spiConfig = hardware::SPI_DEFAULT);
    bool suspend();
    bool resume(hardware::SPIBusConfig spiConfig = hardware::SPI_DEFAULT);
    bool start();
    void stop();
    imu::IMUReport getReport();
};

#endif // IMU_H