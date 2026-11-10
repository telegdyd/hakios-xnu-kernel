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
// Register arrays a6xx.xml defines once, with no A6XX/A7XX variants (Mesa 25.0.7): every element is at the same
// offset on both generations. The table above names only their bases; an element past the base is one of these.
struct SharedArray { uint32_t first, last; };
inline constexpr SharedArray kSharedArrays[] = {
    {0x8010, 0x806f},   // GRAS_CL_VPORT[16], stride 6       a6xx.xml:3084
    {0x8070, 0x808f},   // GRAS_CL_Z_CLAMP[16], stride 2     a6xx.xml:3092
    {0x80b0, 0x80cf},   // GRAS_SC_SCREEN_SCISSOR[16]        a6xx.xml:3296
    {0x80d0, 0x80ef},   // GRAS_SC_VIEWPORT_SCISSOR[16]      a6xx.xml:3300
    {0x8820, 0x885f},   // RB_MRT[8], stride 8               a6xx.xml:3668
    {0x8903, 0x891a},   // RB_MRT_FLAG_BUFFER[8], stride 3   a6xx.xml:3966
    {0x9200, 0x9207},   // VPC_VARYING_INTERP[8]             a6xx.xml:4230
    {0x9208, 0x920f},   // VPC_VARYING_PS_REPL[8]            a6xx.xml:4234
    {0x9212, 0x9215},   // VPC_VAR[4]                        a6xx.xml:4243
    {0xa010, 0xa08f},   // VFD_FETCH[32], stride 4           a6xx.xml:4617
    {0xa090, 0xa0cf},   // VFD_DECODE[32], stride 2          a6xx.xml:4622
    {0xa0d0, 0xa0ef},   // VFD_DEST_CNTL[32]                 a6xx.xml:4635
    {0xa803, 0xa812},   // SP_VS_OUT[16]                     a6xx.xml:4739
    {0xa813, 0xa81a},   // SP_VS_VPC_DST[8]                  a6xx.xml:4754
    {0xa98e, 0xa995},   // SP_FS_OUTPUT[8]                   a6xx.xml:5039
    {0xa996, 0xa99d},   // SP_FS_MRT[8]                      a6xx.xml:5047
};
inline uint32_t offset(const pm4::Stream& stream, uint32_t legacy) {
    if (!stream.gen7()) return legacy;
    uint32_t result = legacyRegisterOffset(kChipA730, legacy);
    if (result == kRegisterAbsent) {
        for (const auto& a : kSharedArrays) {
            if (legacy >= a.first && legacy <= a.last) result = legacy;
        }
    }
    // never emitted as an address: a register the A7XX does not have is the emitter's bug
    if (result == kRegisterAbsent) __builtin_trap();
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
// An A7XX-only register by its A7XX offset (no A6XX name to translate)
void raw(pm4::Stream& stream, uint32_t a7xxOffset, uint32_t value);
// What a 3D pass needs after batchStart: fd6_emit_static_regs(A7XX)'s non-magic part, the VFD fetch sizes zeroed,
// the border colour bases, the A7XX-only tables
// (TRACE: render::setA730Trace's address, for its steps 13-15; 0 for none)
void drawStatic(pm4::Stream& stream, uint64_t borderColors, uint64_t trace = 0);
// A sysmem pass's begin over a W x H window (fd6_emit_sysmem_prep, A7XX)
void sysmemPassBegin(pm4::Stream& stream, uint32_t W, uint32_t H);
void event(pm4::Stream& stream, a6xx::vgt_event_type event, uint64_t address = 0, uint32_t value = 0, bool write = false);
}
