// A7xx-only fields used by the structural generation paths. Mesa 25.0.7.
#pragma once
#include "hakios-metal/a6xx.hpp"
#include "hakios-metal/registers.hpp"
#include "hakios-metal/pm4.hpp"
#include <cassert>
namespace hakios::metal::gen7 {
inline constexpr uint32_t A7XX_SP_CS_CNTL_1_LINEARLOCALIDREGID__MASK = 0x000000ff; // a6xx.xml.h:9294
inline constexpr uint32_t A7XX_SP_CS_CNTL_1_LINEARLOCALIDREGID__SHIFT = 0; // a6xx.xml.h:9295
inline constexpr uint32_t A7XX_SP_CS_CNTL_1_THREADSIZE__MASK = 0x00000100; // a6xx.xml.h:9300
inline constexpr uint32_t A7XX_SP_CS_CNTL_1_THREADSIZE__SHIFT = 8; // a6xx.xml.h:9301
inline constexpr uint32_t A7XX_SP_CS_CNTL_1_THREADSIZE_SCALAR = 0x00000200; // a6xx.xml.h:9306
inline constexpr uint32_t A7XX_SP_CS_CNTL_1_WORKITEMRASTORDER__MASK = 0x00008000; // a6xx.xml.h:9307
inline constexpr uint32_t A7XX_SP_CS_CNTL_1_WORKITEMRASTORDER__SHIFT = 15; // a6xx.xml.h:9308
inline constexpr uint32_t A7XX_HLSQ_CS_CNTL_1_LINEARLOCALIDREGID__MASK = 0x000000ff; // a6xx.xml.h:10869
inline constexpr uint32_t A7XX_HLSQ_CS_CNTL_1_LINEARLOCALIDREGID__SHIFT = 0; // a6xx.xml.h:10870
inline constexpr uint32_t A7XX_HLSQ_CS_CNTL_1_THREADSIZE__MASK = 0x00000200; // a6xx.xml.h:10875
inline constexpr uint32_t A7XX_HLSQ_CS_CNTL_1_THREADSIZE__SHIFT = 9; // a6xx.xml.h:10876
inline constexpr uint32_t A7XX_HLSQ_CS_CNTL_1_WORKGROUPRASTORDERZFIRSTEN = 0x00000800; // a6xx.xml.h:10881
inline constexpr uint32_t A7XX_HLSQ_CS_CNTL_1_WGTILEWIDTH__MASK = 0x03f00000; // a6xx.xml.h:10882
inline constexpr uint32_t A7XX_HLSQ_CS_CNTL_1_WGTILEWIDTH__SHIFT = 20; // a6xx.xml.h:10883
inline constexpr uint32_t A7XX_HLSQ_CS_CNTL_1_WGTILEHEIGHT__MASK = 0xfc000000; // a6xx.xml.h:10888
inline constexpr uint32_t A7XX_HLSQ_CS_CNTL_1_WGTILEHEIGHT__SHIFT = 26; // a6xx.xml.h:10889
inline constexpr uint32_t A7XX_HLSQ_CS_LAST_LOCAL_SIZE_LOCALSIZEX__MASK = 0x00000ffc; // a6xx.xml.h:10897
inline constexpr uint32_t A7XX_HLSQ_CS_LAST_LOCAL_SIZE_LOCALSIZEX__SHIFT = 2; // a6xx.xml.h:10898
inline constexpr uint32_t A7XX_HLSQ_CS_LAST_LOCAL_SIZE_LOCALSIZEY__MASK = 0x003ff000; // a6xx.xml.h:10903
inline constexpr uint32_t A7XX_HLSQ_CS_LAST_LOCAL_SIZE_LOCALSIZEY__SHIFT = 12; // a6xx.xml.h:10904
inline constexpr uint32_t A7XX_HLSQ_CS_LAST_LOCAL_SIZE_LOCALSIZEZ__MASK = 0xffc00000; // a6xx.xml.h:10909
inline constexpr uint32_t A7XX_HLSQ_CS_LAST_LOCAL_SIZE_LOCALSIZEZ__SHIFT = 22; // a6xx.xml.h:10910
inline constexpr uint32_t CP_EVENT_WRITE7_0_EVENT__MASK = 0x000000ff; // adreno_pm4.xml.h:2361
inline constexpr uint32_t CP_EVENT_WRITE7_0_EVENT__SHIFT = 0; // adreno_pm4.xml.h:2362
inline constexpr uint32_t CP_EVENT_WRITE7_0_WRITE_SAMPLE_COUNT = 0x00001000; // adreno_pm4.xml.h:2367
inline constexpr uint32_t CP_EVENT_WRITE7_0_SAMPLE_COUNT_END_OFFSET = 0x00002000; // adreno_pm4.xml.h:2368
inline constexpr uint32_t CP_EVENT_WRITE7_0_WRITE_ACCUM_SAMPLE_COUNT_DIFF = 0x00004000; // adreno_pm4.xml.h:2369
inline constexpr uint32_t CP_EVENT_WRITE7_0_INC_BV_COUNT = 0x00010000; // adreno_pm4.xml.h:2370
inline constexpr uint32_t CP_EVENT_WRITE7_0_INC_BR_COUNT = 0x00020000; // adreno_pm4.xml.h:2371
inline constexpr uint32_t CP_EVENT_WRITE7_0_CLEAR_RENDER_RESOURCE = 0x00040000; // adreno_pm4.xml.h:2372
inline constexpr uint32_t CP_EVENT_WRITE7_0_CLEAR_LRZ_RESOURCE = 0x00080000; // adreno_pm4.xml.h:2373
inline constexpr uint32_t CP_EVENT_WRITE7_0_WRITE_SRC__MASK = 0x00700000; // adreno_pm4.xml.h:2374
inline constexpr uint32_t CP_EVENT_WRITE7_0_WRITE_SRC__SHIFT = 20; // adreno_pm4.xml.h:2375
inline constexpr uint32_t CP_EVENT_WRITE7_0_WRITE_DST__MASK = 0x01000000; // adreno_pm4.xml.h:2380
inline constexpr uint32_t CP_EVENT_WRITE7_0_WRITE_DST__SHIFT = 24; // adreno_pm4.xml.h:2381
inline constexpr uint32_t CP_EVENT_WRITE7_0_WRITE_ENABLED = 0x08000000; // adreno_pm4.xml.h:2386
inline constexpr uint32_t CP_EVENT_WRITE7_0_IRQ = 0x80000000; // adreno_pm4.xml.h:2387
inline constexpr uint32_t CP_EVENT_WRITE7 = 0x46; // adreno_pm4.xml.h:235
inline constexpr uint32_t CP_THREAD_CONTROL = 0x17; // adreno_pm4.xml.h:340
inline constexpr uint32_t CP_THREAD_CONTROL_0_CONCURRENT_BIN_DISABLE = 0x8000000; // adreno_pm4.xml.h:3047
inline constexpr uint32_t CP_SET_THREAD_BR = 0x1; // adreno_pm4.xml.h:604
inline constexpr uint32_t CCU_RESOLVE_CLEAN = 0x1a; // adreno_pm4.xml.h:113
inline constexpr uint32_t CCU_CLEAN_COLOR = 0x21; // adreno_pm4.xml.h:119
inline constexpr uint32_t CCU_CLEAN_DEPTH = 0x20; // adreno_pm4.xml.h:118
inline constexpr uint32_t CACHE_CLEAN = 0x31; // adreno_pm4.xml.h:121
inline constexpr uint32_t CACHE_INVALIDATE7 = 0x33; // adreno_pm4.xml.h:123
struct MagicWrite { uint32_t reg, value; };
inline constexpr MagicWrite kMagic[] = {
#define GPU_MAGIC(reg, value) {reg, value},
#include "hakios-metal/a730-state.inc"
#undef GPU_MAGIC
};

// Shared encoders still name A6xx registers. Multiword blocks used here stay
// contiguous on both generations; the structurally different CS block does not.
inline uint32_t offset(const pm4::Stream& stream, uint32_t legacy) {
    if (!stream.gen7()) return legacy;
    const uint32_t result = legacyRegisterOffset(kChipA730, legacy);
    assert(result != kRegisterAbsent);
    return result;
}
inline void registers(pm4::Stream& stream, uint32_t legacy, const uint32_t* words, uint32_t count) {
    stream.registers(offset(stream, legacy), words, count);
}
inline uint32_t hlsqCsControl(uint32_t threadsize, uint32_t y) {
    const uint32_t height = y % 8 == 0 ? 3 : y % 4 == 0 ? 5 : y % 2 == 0 ? 9 : 17;
    return 0xfc | (threadsize << A7XX_HLSQ_CS_CNTL_1_THREADSIZE__SHIFT) |
           A7XX_HLSQ_CS_CNTL_1_WORKGROUPRASTORDERZFIRSTEN |
           (4u << A7XX_HLSQ_CS_CNTL_1_WGTILEWIDTH__SHIFT) | (height << A7XX_HLSQ_CS_CNTL_1_WGTILEHEIGHT__SHIFT);
}
void batchStart(pm4::Stream& stream);
void event(pm4::Stream& stream, a6xx::vgt_event_type event, uint64_t address = 0, uint32_t value = 0, bool write = false);
}
