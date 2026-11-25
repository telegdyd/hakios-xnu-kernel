// Chip facts shared by the driver and the host model. Mesa 25.0.7's
// freedreno_devices.py (a6xx_gen4, a7xx_base + a7xx_gen1), and HakiosGPU's
// GET_PARAM answers. Knowing a chip here does not enable it in the Metal plug-in.
#pragma once

#include <cstdint>

namespace hakios::metal {

struct ChipInfo {
    uint32_t id, generation, gmemBytes, highestBankBit;
    uint32_t numCcu, depthCacheBytesPerCcu, colorCacheBytesPerCcu;
    uint32_t gmemColorCacheDivisor, tileAlignW, tileAlignH;
    bool earlyPreamble;
};

inline constexpr ChipInfo kChipA635{
    0x06030500, 6, 512 * 1024, 14, 2, 64 * 1024, 64 * 1024, 4, 32, 16, false};
inline constexpr ChipInfo kChipA730{
    0x07030001, 7, 2 * 1024 * 1024, 16, 4, 256 * 1024, 64 * 1024, 8, 64, 32, true};

// The legacy kernel did not always report a chip ID. Preserve that fallback;
// an explicitly unknown chip must never silently acquire another chip's facts.
inline constexpr const ChipInfo* chipInfo(uint32_t id) {
    return id == 0 || id == kChipA635.id ? &kChipA635 : id == kChipA730.id ? &kChipA730 : nullptr;
}

}  // namespace hakios::metal
