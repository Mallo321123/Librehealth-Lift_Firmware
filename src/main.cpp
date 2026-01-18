#include <Arduino.h>
#include <SPI.h>

#include "hardware-drivers/IMU/main.cpp"
#include "hardware-reference.h"
#include "IMU.h"

#include "SD_MMC.h"

IMUSensor imu_obj;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting up...");
  IMUSensor imu_obj = IMUSensor();

  SD_MMC.setPins(14, 15, 2, 4, 12, 13);

  if (!SD_MMC.begin("/sdcard", false , true, 10000))
  {
    Serial.println("SD MMC Mount Failed");
    return;
  }
  Serial.println("SD MMC Mounted!");
}

bool appendLine(const char *path, const char *line)
{
  File file = SD_MMC.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Fehler beim Öffnen der Datei");
    if (!SD_MMC.begin("/sdcard", false, true, 10000))
    {
      Serial.println("SD MMC Mount Failed");
    }
    return false;
  }
  file.println(line); // schreibt Zeile mit Zeilenumbruch
  file.close();

  return true;
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

  if (!appendLine("/log.csv", log_line.c_str()))
  {
    delay(1000);
  }

  // Serial.print(",X:");
  // Serial.print(report.accX);
  // Serial.print(",Y:");
  // Serial.print(report.accY);
  // Serial.print(",Z:");
  // Serial.println(report.accZ);

    delay(1);
}
