#include <Arduino.h>
#include <SPI.h>

#include "hardware-drivers/IMU/main.cpp"
#include "hardware-drivers/SD/main.cpp"
#include "hardware-reference.h"
#include "IMU.h"

IMUSensor imu_obj;
SDCard sd_card;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting up...");

  if (!imu_obj.beggin())
  {
    Serial.println("Failed to initialize IMU!");
  }

  if (!sd_card.begin())
  {
    Serial.println("Failed to initialize SD card!");
  }
  if (sd_card.fileExists("/log.csv"))
  {
    Serial.println("Rotating existing log file...");
    sd_card.rotate_file("/log.csv");
  }
}

void loop()
{
  imu::IMUReport report = imu_obj.update();

  String log_line = String(micros());
  log_line += ",";
  log_line += String(report.accX, 6);
  log_line += ",";
  log_line += String(report.accY, 6);
  log_line += ",";
  log_line += String(report.accZ, 6);

  if (!sd_card.appendLine("/log.csv", log_line.c_str()))
  {
    delay(1000);
  }

  delay(1);
}
