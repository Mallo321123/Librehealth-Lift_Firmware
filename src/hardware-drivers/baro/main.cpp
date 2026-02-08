#include "MS5611.h"
#include "Arduino.h"

#include "drivers/baro.h"
#include "hardware-reference.h"
#include "kalman.h"

void Barometer::valueUpdateTask(void *pvParameters)
{

    Barometer *self = static_cast<Barometer *>(pvParameters);

    for (;;)
    {
        self->ms5611.read();                                          // No thread safety needed, cause this is the only task on this I2C bus
        if (xSemaphoreTake(self->dataMutex, portMAX_DELAY) == pdTRUE) // Wait indefinitely for the mutex to be available
        {
            self->temperature = self->ms5611.getTemperature();
            if (self->config.kalman_filter)
            {
                self->pressure = self->kalmanPressure.update(self->ms5611.getPressure());
            }
            else
            {
                self->pressure = self->ms5611.getPressure();
            }
            xSemaphoreGive(self->dataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(self->config.read_delay_ms));
    }
}

Barometer::Barometer()
{
}

bool Barometer::begin(hardware::baro baroConfig)
{
    Serial.println("Initializing Barometer...");
    ms5611 = MS5611(baroConfig.address);

    dataMutex = xSemaphoreCreateMutex();
    if (!dataMutex)
    {
        Serial.println("Failed to create Barometer mutex!");
        return false;
    }
    state = baro::State::Initialized;

    return true;
}

bool Barometer::resume(hardware::baro baroConfig)
{
    if (state != baro::State::Initialized)
    {
        Serial.println("Barometer not ready for resume!");
        return false;
    }
    Serial.println("Resuming Barometer...");

    // digitalWrite(BARO_PWR_PIN, HIGH); // Power up the sensor

    Wire.begin(baroConfig.sda, baroConfig.scl);
    if (!ms5611.begin())
    {
        Serial.println("Failed to initialize Barometer!");
        return false;
    }
    ms5611.setOversampling(config.osr);
    state = baro::State::Ready;

    return true;
}

bool Barometer::suspend()
{
    if (state != baro::State::Ready)
    {
        Serial.println("Barometer not ready for suspension!");
        return false;
    }
    Serial.println("Suspending Barometer...");

    Wire.end();

    // digitalWrite(BARO_PWR_PIN, LOW); // Power down the sensor

    state = baro::State::Initialized;
    return true;
}

bool Barometer::start()
{
    if (state != baro::State::Ready)
    {
        Serial.println("Barometer not ready for start!");
        return false;
    }
    Serial.println("Starting Barometer...");

    BaseType_t ok1 = xTaskCreate(valueUpdateTask, "BaroValueUpdate", 2048, this, 1, &valueTaskHandle);
    if (ok1 != pdPASS)
    {
        Serial.println("Failed to create Barometer Value Update Task!");
        return false;
    }

    state = baro::State::Running;
    return true;
}

bool Barometer::stop()
{
    if (state != baro::State::Running)
    {
        Serial.println("Barometer not ready for stop!");
        return false;
    }
    Serial.println("Stopping Barometer...");

    vTaskDelete(valueTaskHandle);
    valueTaskHandle = nullptr;

    state = baro::State::Ready;
    return true;
}

float Barometer::getPressure()
{
    if (debug)
    {
        UBaseType_t freeWords = uxTaskGetStackHighWaterMark(valueTaskHandle);
        String debugMsg = "Barometer Value Update Task free stack: " + String(freeWords) + " Bytes";
        Serial.println(debugMsg);
    }

    float p = NAN;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
        p = pressure;
        xSemaphoreGive(dataMutex);
    }
    return p;
}