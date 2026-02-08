#include <Adafruit_ISM330DHCX.h>
#include "SPI.h"
#include "Arduino.h"

#include "hardware-reference.h"
#include "kalman.h"
#include "drivers/IMU.h"

void IMUSensor::applyConfig()
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

    float la_odr_freq = imu::frequency_map[static_cast<int>(config.la_odr)].frequency;
    if (la_odr_freq <= 0.0f)
        la_odr_freq = 100.0f; // Fallback
    update_intervall = static_cast<uint>(max(1.0f, 1000.0f / la_odr_freq));
}

double IMUSensor::mmToG(double mm_s2)
{
    return mm_s2 / 9.80665;
}

void IMUSensor::autorangeCalibrationTask(void *pvParameters)
{
    IMUSensor *self = static_cast<IMUSensor *>(pvParameters);

    // Serial.println("Starting IMU Autorange Calibration Task...");

    for (;;)
    {

        if (xSemaphoreTake(self->configMutex, portMAX_DELAY) == pdTRUE) // Wait indefinitely for the mutex to be available
        {

            if (self->config.accel)
            {
                double max_acc_raw = max(max(fabs(self->report.accX), fabs(self->report.accY)), fabs(self->report.accZ));
                double max_acc = self->mmToG(max_acc_raw * imu::IMU_DEFAULT.autorange_acc_overhead);
                int max_g = imu::accel_range_map[static_cast<int>(self->config.la_fs)].max_g;

                if (max_acc > max_g)
                {
                    if (self->config.la_fs == LSM6DS_ACCEL_RANGE_2_G)
                        self->config.la_fs = LSM6DS_ACCEL_RANGE_4_G;
                    else if (self->config.la_fs == LSM6DS_ACCEL_RANGE_4_G)
                        self->config.la_fs = LSM6DS_ACCEL_RANGE_8_G;
                    else if (self->config.la_fs == LSM6DS_ACCEL_RANGE_8_G)
                        self->config.la_fs = LSM6DS_ACCEL_RANGE_16_G;
                    self->applyConfig();
                }

                if (max_acc_raw < max_g)
                {
                    if (self->config.la_fs == LSM6DS_ACCEL_RANGE_16_G)
                        self->config.la_fs = LSM6DS_ACCEL_RANGE_8_G;
                    else if (self->config.la_fs == LSM6DS_ACCEL_RANGE_8_G)
                        self->config.la_fs = LSM6DS_ACCEL_RANGE_4_G;
                    else if (self->config.la_fs == LSM6DS_ACCEL_RANGE_4_G)
                        self->config.la_fs = LSM6DS_ACCEL_RANGE_2_G;
                    self->applyConfig();
                }
            }

            if (self->config.gyro)
            {
                double max_gyro_raw = max(max(fabs(self->report.gyroX), fabs(self->report.gyroY)), fabs(self->report.gyroZ));
                double max_gyro = max_gyro_raw * imu::IMU_DEFAULT.autorange_gyro_overhead;

                if (max_gyro > self->config.gyro_fs)
                {
                    if (self->config.gyro_fs == LSM6DS_GYRO_RANGE_125_DPS)
                        self->config.gyro_fs = LSM6DS_GYRO_RANGE_250_DPS;
                    else if (self->config.gyro_fs == LSM6DS_GYRO_RANGE_250_DPS)
                        self->config.gyro_fs = LSM6DS_GYRO_RANGE_500_DPS;
                    else if (self->config.gyro_fs == LSM6DS_GYRO_RANGE_500_DPS)
                        self->config.gyro_fs = LSM6DS_GYRO_RANGE_1000_DPS;
                    else if (self->config.gyro_fs == LSM6DS_GYRO_RANGE_1000_DPS)
                        self->config.gyro_fs = LSM6DS_GYRO_RANGE_2000_DPS;
                    else if (self->config.gyro_fs == LSM6DS_GYRO_RANGE_2000_DPS)
                        self->config.gyro_fs = ISM330DHCX_GYRO_RANGE_4000_DPS;
                    self->applyConfig();
                }

                if (max_gyro_raw < self->config.gyro_fs)
                {
                    if (self->config.gyro_fs == ISM330DHCX_GYRO_RANGE_4000_DPS)
                        self->config.gyro_fs = LSM6DS_GYRO_RANGE_2000_DPS;
                    else if (self->config.gyro_fs == LSM6DS_GYRO_RANGE_2000_DPS)
                        self->config.gyro_fs = LSM6DS_GYRO_RANGE_1000_DPS;
                    else if (self->config.gyro_fs == LSM6DS_GYRO_RANGE_1000_DPS)
                        self->config.gyro_fs = LSM6DS_GYRO_RANGE_500_DPS;
                    else if (self->config.gyro_fs == LSM6DS_GYRO_RANGE_500_DPS)
                        self->config.gyro_fs = LSM6DS_GYRO_RANGE_250_DPS;
                    else if (self->config.gyro_fs == LSM6DS_GYRO_RANGE_250_DPS)
                        self->config.gyro_fs = LSM6DS_GYRO_RANGE_125_DPS;
                    self->applyConfig();
                }
            }

            xSemaphoreGive(self->configMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(self->config.range_calibration_update_interval));
    }
}

void IMUSensor::valueUpdateTask(void *pvParameters)
{
    IMUSensor *self = static_cast<IMUSensor *>(pvParameters);

    // Serial.println("Starting IMU Value Update Task...");

    for (;;)
    {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;

        self->ism330dhcx.getEvent(&accel, &gyro, &temp);

        if (xSemaphoreTake(self->reportMutex, portMAX_DELAY) == pdTRUE) // Wait indefinitely for the mutex to be available
        {
            if (self->config.accelKalman)
            {
                self->report.accX = self->kalmanAccX.update(accel.acceleration.x);
                self->report.accY = self->kalmanAccY.update(accel.acceleration.y);
                self->report.accZ = self->kalmanAccZ.update(accel.acceleration.z);
            }
            else
            {
                self->report.accX = accel.acceleration.x;
                self->report.accY = accel.acceleration.y;
                self->report.accZ = accel.acceleration.z;
            }

            if (self->config.gyroKalman)
            {
                self->report.gyroX = self->kalmanGyroX.update(gyro.gyro.x);
                self->report.gyroY = self->kalmanGyroY.update(gyro.gyro.y);
                self->report.gyroZ = self->kalmanGyroZ.update(gyro.gyro.z);
            }
            else
            {
                self->report.gyroX = gyro.gyro.x;
                self->report.gyroY = gyro.gyro.y;
                self->report.gyroZ = gyro.gyro.z;
            }
            xSemaphoreGive(self->reportMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(self->update_intervall));
    }
}

void IMUSensor::calibrateGravity(int samples)
{
    Serial.println("Calibrating IMU... Keep it still!");
    double sumAccX = 0.0, sumAccY = 0.0, sumAccZ = 0.0;

    if (!reportMutex)
        return;

    for (int i = 0; i < samples; i++)
    {
        if (xSemaphoreTake(reportMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            sumAccX += report.accX;
            sumAccY += report.accY;
            sumAccZ += report.accZ;
            xSemaphoreGive(reportMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    gravityOffsetX = sumAccX / samples;
    gravityOffsetY = sumAccY / samples;
    gravityOffsetZ = sumAccZ / samples;

    calibrated = true;

    Serial.println("IMU Calibration complete!");
}

IMUSensor::IMUSensor()
{
}

bool IMUSensor::begin(hardware::SPIBusConfig spiConfig)
{
    Serial.println("Initializing IMU...");
    ism330dhcx = Adafruit_ISM330DHCX();

    reportMutex = xSemaphoreCreateMutex();
    configMutex = xSemaphoreCreateMutex();
    if (!reportMutex || !configMutex)
    {
        Serial.println("Failed to create IMU mutexes!");
        return false;
    }

    state = imu::State::Initialized;
    return true;
}

bool IMUSensor::suspend()
{
    if (state != imu::State::Ready)
    {
        Serial.println("IMU not ready for suspension!");
        return false;
    }

    state = imu::State::Initialized;

    // DigitalWrite(IMU_PWR_PIN, LOW); // Power down the sensor

    Serial.println("IMU suspended!");
    return true;
}

bool IMUSensor::resume(hardware::SPIBusConfig spiConfig)
{
    Serial.println("Resuming IMU...");
    if (state != imu::State::Initialized)
    {
        Serial.println("IMU not ready for resuming!");
        return false;
    }

    if (!ism330dhcx.begin_SPI(spiConfig.cs, spiConfig.sclk, spiConfig.miso, spiConfig.mosi))
    {
        Serial.println("Failed to initialize IMU!");
        return false;
    }

    applyConfig();
    state = imu::State::Ready;
    Serial.println("IMU resumed!");
    return true;
}

bool IMUSensor::start()
{
    if (state != imu::State::Ready)
    {
        Serial.println("IMU not ready for startup!");
        return false;
    }
    Serial.println("Starting IMU Tasks...");

    BaseType_t ok1 = xTaskCreate(valueUpdateTask, "IMU Value Update", 1024, this, 3, &valueTaskHandle);
    BaseType_t ok2 = xTaskCreate(autorangeCalibrationTask, "IMU Autorange Calibration", 1024, this, 2, &autorangeTaskHandle);

    if (ok1 != pdPASS || ok2 != pdPASS)
    {
        Serial.println("Failed to start IMU tasks!");
        return false;
    }

    Serial.println("IMU Tasks Started...");

    calibrateGravity();
    state = imu::State::Running;
    return true;
}

void IMUSensor::stop()
{
    if (autorangeTaskHandle)
    {
        vTaskDelete(autorangeTaskHandle);
        autorangeTaskHandle = nullptr;
    }

    if (valueTaskHandle)
    {
        vTaskDelete(valueTaskHandle);
        valueTaskHandle = nullptr;
    }

    Serial.println("IMU tasks stopped!");
    state = imu::State::Ready;
}

imu::IMUReport IMUSensor::getReport()
{
    if (debug)
    {
        UBaseType_t freeWords = uxTaskGetStackHighWaterMark(valueTaskHandle);
        String debugMsg = "IMU Value Update Task free stack: " + String(freeWords) + " Bytes";
        Serial.println(debugMsg);

        freeWords = uxTaskGetStackHighWaterMark(autorangeTaskHandle);
        debugMsg = "IMU Autorange Calibration Task free stack: " + String(freeWords) + " Bytes";
        Serial.println(debugMsg);
    }

    if (reportMutex && xSemaphoreTake(reportMutex, portMAX_DELAY) == pdTRUE)
    {
        imu::IMUReport currentReport = report;
        xSemaphoreGive(reportMutex);
        return currentReport - imu::IMUReport{gravityOffsetX, gravityOffsetY, gravityOffsetZ, 0.0, 0.0, 0.0};
    }
    return report; // Fallback
}
