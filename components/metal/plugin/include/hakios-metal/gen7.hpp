// A7xx-only fields used by the structural generation paths. Mesa 25.0.7.
#pragma once
#include "hakios-metal/a6xx.hpp"
#include "hakios-metal/registers.hpp"
#include "hakios-metal/pm4.hpp"
#include <array>
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
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_SRC__MASK = 0x0000007f; // a6xx.xml.h:9033
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_SRC__SHIFT = 0; // a6xx.xml.h:9034
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_SAMP_ID__MASK = 0x00000380; // a6xx.xml.h:9039
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_TEX_ID__MASK = 0x00001c00; // a6xx.xml.h:9045
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_DST__MASK = 0x0007e000; // a6xx.xml.h:9051
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_DST__SHIFT = 13; // a6xx.xml.h:9052
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_WRMASK__MASK = 0x00780000; // a6xx.xml.h:9057
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_WRMASK__SHIFT = 19; // a6xx.xml.h:9058
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_HALF = 0x00800000; // a6xx.xml.h:9063
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_BINDLESS = 0x02000000; // a6xx.xml.h:9064
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_CMD__MASK = 0x3c000000; // a6xx.xml.h:9065
inline constexpr uint32_t A7XX_SP_FS_PREFETCH_CMD_CMD__SHIFT = 26; // a6xx.xml.h:9066
inline constexpr uint32_t A6XX_SP_FS_BINDLESS_PREFETCH_CMD_SAMP_ID__MASK = 0x0000ffff; // a6xx.xml.h:9076
inline constexpr uint32_t A6XX_SP_FS_BINDLESS_PREFETCH_CMD_TEX_ID__SHIFT = 16; // a6xx.xml.h:9083
// SP_FS_BINDLESS_PREFETCH_CMD[0..3] (a6xx.xml.h:9075): on the A7XX the prefetch's sampler and texture slots live here,
// bindless or not, and the CMD word's own SAMP_ID/TEX_ID stay 0 (fd6_program.cc emit_fs_inputs)
inline constexpr uint32_t REG_A6XX_SP_FS_BINDLESS_PREFETCH_CMD = 0x0000a9a3; // a6xx.xml.h:9075
// the A7XX's SP_FS_PREFETCH_CMD (its own field positions; slots 0)
constexpr uint32_t spFsPrefetchCmd(uint32_t src, uint32_t dst, uint32_t wrmask, bool half, uint32_t cmd) {
    return ((src << A7XX_SP_FS_PREFETCH_CMD_SRC__SHIFT) & A7XX_SP_FS_PREFETCH_CMD_SRC__MASK) |
           ((dst << A7XX_SP_FS_PREFETCH_CMD_DST__SHIFT) & A7XX_SP_FS_PREFETCH_CMD_DST__MASK) |
           ((wrmask << A7XX_SP_FS_PREFETCH_CMD_WRMASK__SHIFT) & A7XX_SP_FS_PREFETCH_CMD_WRMASK__MASK) | (half ? A7XX_SP_FS_PREFETCH_CMD_HALF : 0u) |
           ((cmd << A7XX_SP_FS_PREFETCH_CMD_CMD__SHIFT) & A7XX_SP_FS_PREFETCH_CMD_CMD__MASK);
}
constexpr uint32_t spFsBindlessPrefetchCmd(uint32_t sampId, uint32_t texId) {
    return (sampId & A6XX_SP_FS_BINDLESS_PREFETCH_CMD_SAMP_ID__MASK) | texId << A6XX_SP_FS_BINDLESS_PREFETCH_CMD_TEX_ID__SHIFT;
}
inline constexpr uint32_t A7XX_RB_DEPTH_BUFFER_INFO_TILEMODE__MASK = 0x00000060; // a6xx.xml.h:5963
inline constexpr uint32_t A7XX_RB_DEPTH_BUFFER_INFO_TILEMODE__SHIFT = 5; // a6xx.xml.h:5964
inline constexpr uint32_t A7XX_RB_STENCIL_INFO_TILEMODE__MASK = 0x0000000c; // a6xx.xml.h:6082
inline constexpr uint32_t A7XX_RB_STENCIL_INFO_TILEMODE__SHIFT = 2; // a6xx.xml.h:6083
constexpr uint32_t A7XX_RB_DEPTH_BUFFER_INFO_TILEMODE(uint32_t tile) { return (tile << A7XX_RB_DEPTH_BUFFER_INFO_TILEMODE__SHIFT) & A7XX_RB_DEPTH_BUFFER_INFO_TILEMODE__MASK; }
constexpr uint32_t A7XX_RB_STENCIL_INFO_TILEMODE(uint32_t tile) { return (tile << A7XX_RB_STENCIL_INFO_TILEMODE__SHIFT) & A7XX_RB_STENCIL_INFO_TILEMODE__MASK; }
inline constexpr uint32_t A7XX_HLSQ_INVALIDATE_CMD_GFX_BINDLESS__MASK = 0x01fe0000; // a6xx.xml.h:11065
inline constexpr uint32_t A7XX_HLSQ_INVALIDATE_CMD_GFX_BINDLESS__SHIFT = 17; // a6xx.xml.h:11066
constexpr uint32_t A7XX_HLSQ_INVALIDATE_CMD_GFX_BINDLESS(uint32_t sets) {
    return (sets << A7XX_HLSQ_INVALIDATE_CMD_GFX_BINDLESS__SHIFT) & A7XX_HLSQ_INVALIDATE_CMD_GFX_BINDLESS__MASK;
}
// SP_BINDLESS_BASE[0] of the graphics stages on the A7XX (a6xx.xml.h:9494; the A6XX's is 0xab10), its low bits the
// descriptor size: BINDLESS_DESCRIPTOR_64B, as compute.cpp's SP_CS_BINDLESS_BASE
inline constexpr uint32_t REG_A7XX_SP_BINDLESS_BASE = 0x0000ab0a; // a6xx.xml.h:9494
inline constexpr uint32_t BINDLESS_DESCRIPTOR_64B = 3; // a6xx.xml.h:2317
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
inline constexpr uint32_t kLegacyRegisterCount = 0xc000;
inline constexpr uint16_t kLegacyRegisterAbsent = 0xffff;
static_assert([] {
    for (const auto& r : kRegisters) {
        if (r.a6xx != kRegisterAbsent && r.a6xx >= kLegacyRegisterCount) return false;
        if (r.a7xx != kRegisterAbsent && r.a7xx >= kLegacyRegisterAbsent) return false;
    }
    for (const auto& a : kSharedArrays) {
        if (a.first > a.last || a.last >= kLegacyRegisterCount) return false;
    }
    return true;
}(), "A7xx legacy register table bounds or sentinel need updating");

inline constexpr std::array<uint16_t, kLegacyRegisterCount> kLegacyRegisterOffsets = [] {
    std::array<uint16_t, kLegacyRegisterCount> offsets{};
    for (auto& entry : offsets) entry = kLegacyRegisterAbsent;

    // Walk backwards so the first named entry wins if an A6xx offset is aliased.
    for (auto i = static_cast<uint32_t>(Register::Count); i > 0; --i) {
        const auto& r = kRegisters[i - 1];
        if (r.a6xx == kRegisterAbsent) continue;
        offsets[r.a6xx] = r.a7xx == kRegisterAbsent ? kLegacyRegisterAbsent : static_cast<uint16_t>(r.a7xx);
    }
    for (const auto& a : kSharedArrays) {
        for (uint32_t legacy = a.first; legacy <= a.last; ++legacy) {
            if (offsets[legacy] == kLegacyRegisterAbsent) offsets[legacy] = static_cast<uint16_t>(legacy);
        }
    }
    return offsets;
}();

inline uint32_t offset(const pm4::Stream& stream, uint32_t legacy) {
    if (!stream.gen7()) return legacy;
    if (legacy >= kLegacyRegisterCount) __builtin_trap();
    const uint16_t result = kLegacyRegisterOffsets[legacy];
    // never emitted as an address: a register the A7XX does not have is the emitter's bug
    if (result == kLegacyRegisterAbsent) __builtin_trap();
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

// A7XX-only registers of tiled rendering (a6xx.xml.h)
inline constexpr uint32_t REG_A7XX_RB_CCU_CNTL2 = 0x88e5;          // a6xx.xml.h:6418
inline constexpr uint32_t REG_A7XX_RB_BLIT_CLEAR_MODE = 0x88e4;    // a6xx.xml.h:6409  CLEAR_MODE_SYSMEM 0, CLEAR_MODE_GMEM 1
inline constexpr uint32_t REG_A7XX_GRAS_SU_RENDER_CNTL = 0x8116;
inline constexpr uint32_t REG_A7XX_VSC_UNKNOWN_0D08 = 0x0d08;      // a6xx.xml.h:4244
// CP_SET_PSEUDO_REG (adreno_pm4.xml.h:291) and the pseudo registers the A7XX's VSC streams are named by (:538-540):
// three words each, the register, then its 64-bit address
inline constexpr uint32_t CP_SET_PSEUDO_REG = 86;
inline constexpr uint32_t kPseudoDrawStrmAddress = 8, kPseudoDrawStrmSizeAddress = 9, kPseudoPrimStrmAddress = 10;
// RB_RENDER_CNTL's and GRAS_SU_RENDER_CNTL's BINNING (A7XX): the binning pass; 0 in both is a tile (TYPE_TILED) or sysmem
inline constexpr uint32_t kRenderCntlBinning = 0x80;
// RB_CCU_CNTL2 (A7XX: the CCU split RB_CCU_CNTL held on A6XX): in sysmem the depth caches at 0 and the colour caches
// after them, all full size (0x80000000 on the A730); for tiles the colour cache shrunk to its GMEM fraction at
// gmemCcuColorOffset, the depth cache unmoved (emit_rb_ccu_cntl / fd6_emit_ccu_cntl<A7XX>: 0xfc600000 on the A730)
constexpr uint32_t rbCcuCntl2(const ChipInfo& c, bool gmem) {
    const uint32_t color = gmem ? gmemCcuColorOffset(c) : c.numCcu * c.depthCacheBytesPerCcu;
    uint32_t fraction = 0;   // (CCU_CACHE_SIZE_FULL, _HALF, _QUARTER, _EIGHTH: the divisor's log2)
    for (uint32_t d = gmem ? c.gmemColorCacheDivisor : 1; d > 1; d >>= 1) fraction++;
    return ((color >> 21) & 1u) << 2 | fraction << 21 | ((color & 0x1fffff) >> 12) << 23;
}
static_assert(rbCcuCntl2(kChipA730, false) == 0x80000000 && rbCcuCntl2(kChipA730, true) == 0xfc600000, "freedreno's FD730 values");
// The A7XX registers that say where the buffers are: for tiles (GMEM) tu7_emit_tile_render_begin_regs' --
// RB_UNKNOWN_8812 0, RB_UNKNOWN_8E06 0, GRAS_UNKNOWN_8007 0, GRAS_UNKNOWN_8110 2, RB_UNKNOWN_8E09 4, RB_BLIT_CLEAR_MODE GMEM;
// for sysmem what sysmemPassBegin writes (RB_UNKNOWN_8812 0x3ff, the A730's RB_UNKNOWN_8E06) and RB_BLIT_CLEAR_MODE SYSMEM
void tileRegisters(pm4::Stream& stream, bool gmem);
}
