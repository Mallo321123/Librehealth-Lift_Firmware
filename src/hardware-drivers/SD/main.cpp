#include "SD_MMC.h"

#include "SD.h"
#include "hardware-reference.h"

class SDCard
{
private:
    sd::State state = sd::State::Uninitialized;
    sd::SDConfig sdConfig = sd::SD_DEFAULT;

public:
    SDCard()
    {
    }

    bool begin(const hardware::SDCardConfig &hwConfig = hardware::SDCARD_DEFAULT)
    {
        Serial.println("Initializing SD Card...");
        SD_MMC.setPins(
            hwConfig.clk,
            hwConfig.cmd,
            hwConfig.d0,
            hwConfig.d1,
            hwConfig.d2,
            hwConfig.d3);

        if (!SD_MMC.begin(
                sdConfig.mountPoint,
                sdConfig.OneBitMode,
                sdConfig.formatIfMountFailed,
                sdConfig.sdmmcFrequencyMHz))
        {
            state = sd::State::Uninitialized;
            Serial.println("SD MMC Mount Failed");
            return false;
        }

        Serial.println("SD MMC Mounted!");
        state = sd::State::Ready;
        return true;
    }

    bool appendLine(const char *path, const char *line)
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

    bool rotate_file(const char *path)
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

    bool fileExists(const char *path)
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
};
