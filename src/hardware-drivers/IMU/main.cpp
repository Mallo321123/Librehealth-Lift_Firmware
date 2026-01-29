#include <Adafruit_ISM330DHCX.h>
#include "SPI.h"
#include "Arduino.h"

#include "hardware-reference.h"
#include "IMU.h"
#include "drivers/kalmanFilter.cpp"

class IMUSensor
{
private:
    Adafruit_ISM330DHCX ism330dhcx;

    imu::State state = imu::State::Uninitialized;

    unsigned long last_accel_read = 0;
    unsigned long last_gyro_read = 0;

    uint accel_update_intervall;
    uint gyro_update_intervall;

    KalmanFilter kalmanAccX{KalmanFilter::accel};
    KalmanFilter kalmanAccY{KalmanFilter::accel};
    KalmanFilter kalmanAccZ{KalmanFilter::accel};

    KalmanFilter kalmanGyroX{KalmanFilter::gyro};
    KalmanFilter kalmanGyroY{KalmanFilter::gyro};
    KalmanFilter kalmanGyroZ{KalmanFilter::gyro};

    imu::IMUConfig config = imu::IMU_DEFAULT;
    void (*configChangedCallback)(const imu::IMUConfig &) = nullptr;

    void applyConfig()
    {
        if (state != imu::State::Initialized)
        {
            return;
        }

        ism330dhcx.setAccelRange(config.la_fs);
        ism330dhcx.setGyroRange(config.gyro_fs);
        ism330dhcx.setAccelDataRate(config.la_odr);
        ism330dhcx.setGyroDataRate(config.gyro_odr);

        config.accel = (config.la_odr != LSM6DS_RATE_SHUTDOWN);
        config.gyro = (config.gyro_odr != LSM6DS_RATE_SHUTDOWN);

        if (config.accel)
        {
            float la_odr_freq = imu::frequency_map[static_cast<int>(config.la_odr)].frequency;
            accel_update_intervall = 1000000 / la_odr_freq;
        }

        if (config.gyro)
        {
            float gyro_odr_freq = imu::frequency_map[static_cast<int>(config.gyro_odr)].frequency;
            gyro_update_intervall = 1000000 / gyro_odr_freq;
        }
    }

    double mmToG(double mm_s2)
    {
        return mm_s2 / 9.80665;
    }

    void autorangeCalibration(const imu::IMUReport &report)
    {
        if (config.accel)
        {
            double max_acc;
            double max_acc_raw;

            max_acc_raw = max(max(fabs(report.accX), fabs(report.accY)), fabs(report.accZ));
            max_acc = max_acc_raw * imu::IMU_DEFAULT.autorange_acc_overhead;

            max_acc = mmToG(max_acc);

            int max_g = imu::accel_range_map[static_cast<int>(config.la_fs)].max_g;

            if (max_acc > max_g)
            {
                // Increase range
                if (config.la_fs == LSM6DS_ACCEL_RANGE_2_G)
                {
                    config.la_fs = LSM6DS_ACCEL_RANGE_4_G;
                }
                else if (config.la_fs == LSM6DS_ACCEL_RANGE_4_G)
                {
                    config.la_fs = LSM6DS_ACCEL_RANGE_8_G;
                }
                else if (config.la_fs == LSM6DS_ACCEL_RANGE_8_G)
                {
                    config.la_fs = LSM6DS_ACCEL_RANGE_16_G;
                }
                applyConfig();
            }

            if (max_acc_raw < max_g)
            {
                // Decrease range
                if (config.la_fs == LSM6DS_ACCEL_RANGE_16_G)
                {
                    config.la_fs = LSM6DS_ACCEL_RANGE_8_G;
                }
                else if (config.la_fs == LSM6DS_ACCEL_RANGE_8_G)
                {
                    config.la_fs = LSM6DS_ACCEL_RANGE_4_G;
                }
                else if (config.la_fs == LSM6DS_ACCEL_RANGE_4_G)
                {
                    config.la_fs = LSM6DS_ACCEL_RANGE_2_G;
                }
                applyConfig();
            }
        }

        if (config.gyro)
        {
            double max_gyro;
            double max_gyro_raw;

            max_gyro_raw = max(max(fabs(report.gyroX), fabs(report.gyroY)), fabs(report.gyroZ));
            max_gyro = max_gyro_raw * imu::IMU_DEFAULT.autorange_gyro_overhead;

            if (max_gyro > config.gyro_fs)
            {
                // Increase range
                if (config.gyro_fs == LSM6DS_GYRO_RANGE_125_DPS)
                {
                    config.gyro_fs = LSM6DS_GYRO_RANGE_250_DPS;
                }
                else if (config.gyro_fs == LSM6DS_GYRO_RANGE_250_DPS)
                {
                    config.gyro_fs = LSM6DS_GYRO_RANGE_500_DPS;
                }
                else if (config.gyro_fs == LSM6DS_GYRO_RANGE_500_DPS)
                {
                    config.gyro_fs = LSM6DS_GYRO_RANGE_1000_DPS;
                }
                else if (config.gyro_fs == LSM6DS_GYRO_RANGE_1000_DPS)
                {
                    config.gyro_fs = LSM6DS_GYRO_RANGE_2000_DPS;
                }
                else if (config.gyro_fs == LSM6DS_GYRO_RANGE_2000_DPS)
                {
                    config.gyro_fs = ISM330DHCX_GYRO_RANGE_4000_DPS;
                }

                applyConfig();
            }

            if (max_gyro_raw < config.gyro_fs)
            {
                // Decrease range
                if (config.gyro_fs == ISM330DHCX_GYRO_RANGE_4000_DPS)
                {
                    config.gyro_fs = LSM6DS_GYRO_RANGE_2000_DPS;
                }
                else if (config.gyro_fs == LSM6DS_GYRO_RANGE_2000_DPS)
                {
                    config.gyro_fs = LSM6DS_GYRO_RANGE_1000_DPS;
                }
                else if (config.gyro_fs == LSM6DS_GYRO_RANGE_1000_DPS)
                {
                    config.gyro_fs = LSM6DS_GYRO_RANGE_500_DPS;
                }
                else if (config.gyro_fs == LSM6DS_GYRO_RANGE_500_DPS)
                {
                    config.gyro_fs = LSM6DS_GYRO_RANGE_250_DPS;
                }
                else if (config.gyro_fs == LSM6DS_GYRO_RANGE_250_DPS)
                {
                    config.gyro_fs = LSM6DS_GYRO_RANGE_125_DPS;
                }
                applyConfig();
            }
        }
    }

public:
    IMUSensor()
    {
    }

    bool beggin(hardware::SPIBusConfig spiConfig = hardware::SPI_DEFAULT)
    {
        Serial.println("Initializing IMU...");
        ism330dhcx = Adafruit_ISM330DHCX();
        if (!ism330dhcx.begin_SPI(spiConfig.cs, spiConfig.sclk, spiConfig.miso, spiConfig.mosi))
        {
            Serial.println("Failed to initialize IMU!");
            state = imu::State::Uninitialized;
            return false;
        }
        Serial.println("IMU initialized!");

        state = imu::State::Initialized;
        applyConfig();
        state = imu::State::Ready;
        return true;
    }

    imu::IMUReport update()
    {
        imu::IMUReport report;

        if (state != imu::State::Ready)
        {
            return report;
        }

        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;

        ism330dhcx.getEvent(&accel, &gyro, &temp);

        autorangeCalibration(imu::IMUReport{
            .accX = accel.acceleration.x,
            .accY = accel.acceleration.y,
            .accZ = accel.acceleration.z,
            .gyroX = gyro.gyro.x,
            .gyroY = gyro.gyro.y,
            .gyroZ = gyro.gyro.z,
        });

        if (last_accel_read + accel_update_intervall < micros() && config.accelKalman)
        {
            report.accX = kalmanAccX.update(accel.acceleration.x);
            report.accY = kalmanAccY.update(accel.acceleration.y);
            report.accZ = kalmanAccZ.update(accel.acceleration.z);

            last_accel_read = micros();
        }
        else
        {
            report.accX = accel.acceleration.x;
            report.accY = accel.acceleration.y;
            report.accZ = accel.acceleration.z;
        }

        if (last_gyro_read + gyro_update_intervall < micros() && config.gyroKalman)
        {
            report.gyroX = kalmanGyroX.update(gyro.gyro.x);
            report.gyroY = kalmanGyroY.update(gyro.gyro.y);
            report.gyroZ = kalmanGyroZ.update(gyro.gyro.z);

            last_gyro_read = micros();
        }
        else
        {

            report.gyroX = gyro.gyro.x;
            report.gyroY = gyro.gyro.y;
            report.gyroZ = gyro.gyro.z;
        }

        // if (config.la_fs == LSM6DS_ACCEL_RANGE_2_G)
        //     Serial.print("accel_range:19.62,");
        // else if (config.la_fs == LSM6DS_ACCEL_RANGE_4_G)
        //     Serial.print("accel_range:39.24,");
        // else if (config.la_fs == LSM6DS_ACCEL_RANGE_8_G)
        //     Serial.print("accel_range:78.48,");
        // else if (config.la_fs == LSM6DS_ACCEL_RANGE_16_G)
        //     Serial.print("accel_range:156.96,");

        // if (config.gyro_fs == LSM6DS_GYRO_RANGE_125_DPS)
        //     Serial.print("gyro_range:125DPS");
        // else if (config.gyro_fs == LSM6DS_GYRO_RANGE_250_DPS)
        //     Serial.print("gyro_range:250DPS");
        // else if (config.gyro_fs == LSM6DS_GYRO_RANGE_500_DPS)
        //     Serial.print("gyro_range:500DPS");
        // else if (config.gyro_fs == LSM6DS_GYRO_RANGE_1000_DPS)
        //     Serial.print("gyro_range:1000DPS");
        // else if (config.gyro_fs == LSM6DS_GYRO_RANGE_2000_DPS)
        //     Serial.print("gyro_range:2000DPS");
        // else if (config.gyro_fs == ISM330DHCX_GYRO_RANGE_4000_DPS)
        //     Serial.print("gyro_range:4000DPS");

        return report;
    }
};
