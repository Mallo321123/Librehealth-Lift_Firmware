#ifndef SD_H
#define SD_H

#include "hardware-reference.h"

namespace sd
{

    struct SDConfig
    {
        const char *mountPoint;
        bool OneBitMode = false;
        bool formatIfMountFailed = true;
        int sdmmcFrequencyMHz = 40000;

        constexpr SDConfig(char *mountPoint_ = "/sdcard",
                           bool OneBitMode_ = false,
                           bool formatIfMountFailed_ = true,
                           int sdmmcFrequencyMHz_ = 40000) noexcept
            : mountPoint(mountPoint_),
              OneBitMode(OneBitMode_),
              formatIfMountFailed(formatIfMountFailed_),
              sdmmcFrequencyMHz(sdmmcFrequencyMHz_) {}
    };

    enum class State
    {
        Uninitialized,
        Initialized,
        Ready,
    };

    static constexpr SDConfig SD_DEFAULT;

};

class SDCard
{
private:
    sd::State state = sd::State::Uninitialized;
    sd::SDConfig sdConfig = sd::SD_DEFAULT;

public:
    SDCard();
    bool begin(const hardware::SDCardConfig &hwConfig = hardware::SDCARD_DEFAULT);
    bool resume();
    bool suspend();
    bool appendLine(const char *path, const char *line);
    bool rotate_file(const char *path);
    bool fileExists(const char *path);
};

#endif // SD_H