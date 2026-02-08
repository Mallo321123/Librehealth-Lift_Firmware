#include "SD_MMC.h"

#include "drivers/SD.h"
#include "hardware-reference.h"

SDCard::SDCard()
{
}

bool SDCard::begin(const hardware::SDCardConfig &hwConfig)
{
    Serial.println("Initializing SD Card...");
    SD_MMC.setPins(
        hwConfig.clk,
        hwConfig.cmd,
        hwConfig.d0,
        hwConfig.d1,
        hwConfig.d2,
        hwConfig.d3);

    state = sd::State::Initialized;
    return true;
}

bool SDCard::resume()
{
    if (state != sd::State::Initialized)
    {
        Serial.println("SD Card not ready for resuming!");
        return false;
    }

    // digitalWrite(SD_PWR_PIN, HIGH); // Power up the SD card

    if (!SD_MMC.begin(
            sdConfig.mountPoint,
            sdConfig.OneBitMode,
            sdConfig.formatIfMountFailed,
            sdConfig.sdmmcFrequencyMHz))
    {
        state = sd::State::Initialized;
        Serial.println("SD MMC Mount Failed");
        return false;
    }
    state = sd::State::Ready;
    Serial.println("SD Card resumed!");
    return true;
}

bool SDCard::suspend()
{
    if (state != sd::State::Ready)
    {
        Serial.println("SD Card not ready for suspension!");
        return false;
    }

    SD_MMC.end();
    state = sd::State::Initialized;

    // digitalWrite(SD_PWR_PIN, LOW); // Power down the SD card

    Serial.println("SD Card suspended!");
    return true;
}

bool SDCard::appendLine(const char *path, const char *line)
{
    if (state != sd::State::Ready)
    {
        if (!begin())
        {
            return false;
        }
    }

    File file = SD_MMC.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Fehler beim Öffnen der Datei");
        return false;
    }

    file.println(line);
    file.close();
    return true;
}

bool SDCard::rotate_file(const char *path)
{
    if (state != sd::State::Ready)
    {
        if (!begin())
        {
            return false;
        }
    }

    if (!SD_MMC.exists(path))
    {
        return true;
    }

    char old_path[128];
    char next_path[128];

    strncpy(old_path, path, sizeof(old_path));
    old_path[sizeof(old_path) - 1] = '\0';

    while (true)
    {
        snprintf(next_path, sizeof(next_path), "%s.old", old_path);

        if (!SD_MMC.exists(next_path))
        {
            break;
        }
        Serial.print("Datei existiert bereits: ");
        Serial.println(next_path);

        strncpy(old_path, next_path, sizeof(old_path));
        old_path[sizeof(old_path) - 1] = '\0';
    }

    Serial.print("Umbenennen von ");
    Serial.print(path);
    Serial.print(" nach ");
    Serial.println(next_path);

    if (!SD_MMC.rename(path, next_path))
    {
        Serial.println("Fehler beim Umbenennen der Datei");
        return false;
    }

    return true;
}

bool SDCard::fileExists(const char *path)
{
    if (state != sd::State::Ready)
    {
        if (!begin())
        {
            return false;
        }
    }

    return SD_MMC.exists(path);
}
