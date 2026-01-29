#ifndef SD_H
#define SD_H

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
        Ready
    };

    static constexpr SDConfig SD_DEFAULT;

}

#endif // SD_H