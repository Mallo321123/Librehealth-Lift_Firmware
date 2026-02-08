#include <Arduino.h>
#include <SPI.h>

#include "drivers/SD.h"
#include "drivers/baro.h"
#include "drivers/IMU.h"
#include "hardware-reference.h"

IMUSensor imu_obj;
Barometer baro_obj;
SDCard sd_card;

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting up...");

  if (!imu_obj.begin())
  {
    Serial.println("Failed to initialize IMU!");
  }

  if (!imu_obj.resume())
  {
    Serial.println("Failed to start IMU!");
  }

  if (!imu_obj.start())
  {
    Serial.println("Failed to start IMU tasks!");
  }

  if (!baro_obj.begin())
  {
    Serial.println("Failed to initialize Barometer!");
  }
  if (!baro_obj.resume())
  {
    Serial.println("Failed to start Barometer!");
  }
  if (!baro_obj.start())
  {
    Serial.println("Failed to start Barometer tasks!");
  }

  if (!sd_card.begin())
  {
    Serial.println("Failed to initialize SD card!");
  }
  if (!sd_card.resume())
  {
    Serial.println("Failed to resume SD card!");
  }

  if (sd_card.fileExists("/log.csv"))
  {
    Serial.println("Rotating existing log file...");
    sd_card.rotate_file("/log.csv");
  }

  String header = "time,accX,accY,accZ,pressure";
  if (!sd_card.appendLine("/log.csv", header.c_str())) 
  {
    Serial.println("Failed to write header to log file!");
  }
}
void loop()
{
  imu::IMUReport report = imu_obj.getReport();

  // CSV-Zeile: Zeit, Roh-Acc (XYZ), Komp-Acc (XYZ), Geschw (XYZ)
  String log_line = String(millis());
  log_line += ",";
  log_line += String(report.accX, 6);
  log_line += ",";
  log_line += String(report.accY, 6);
  log_line += ",";
  log_line += String(report.accZ, 6);
  log_line += ",";
  log_line += String(baro_obj.getPressure(), 2);

  if (!sd_card.appendLine("/log.csv", log_line.c_str()))
  {
    Serial.println("Failed to write log line to SD card!");
  }

  delayMicroseconds(500);
}