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
    // the largest bin (freedreno_devices.py A6xxGPUInfo: 1024 x 1008 on A6XX, the BIN_CONTROL fields' limits; 1728 x
    // 1728 on A7XX)
    uint32_t tileMaxW, tileMaxH;
};

inline constexpr ChipInfo kChipA635{
    0x06030500, 6, 512 * 1024, 14, 2, 64 * 1024, 64 * 1024, 4, 32, 16, false, 1024, 1008};
inline constexpr ChipInfo kChipA730{
    0x07030001, 7, 2 * 1024 * 1024, 16, 4, 256 * 1024, 64 * 1024, 8, 64, 32, true, 1728, 1728};

// The legacy kernel did not always report a chip ID. Preserve that fallback;
// an explicitly unknown chip must never silently acquire another chip's facts.
inline constexpr const ChipInfo* chipInfo(uint32_t id) {
    return id == 0 || id == kChipA635.id ? &kChipA635 : id == kChipA730.id ? &kChipA730 : nullptr;
}

// Where the colour CCU sits in GMEM while tiles are drawn (tu_device.cc ccu_offset_gmem: GMEM's end less the colour
// cache shrunk to its GMEM fraction); the bins own everything below it. 0x78000 on the A635, 0x1f8000 on the A730.
inline constexpr uint32_t gmemCcuColorOffset(const ChipInfo& c) {
    return c.gmemBytes - c.numCcu * c.colorCacheBytesPerCcu / c.gmemColorCacheDivisor;
}
static_assert(gmemCcuColorOffset(kChipA635) == 0x78000 && gmemCcuColorOffset(kChipA730) == 0x1f8000);

}  // namespace hakios::metal
