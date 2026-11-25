#include "hakios-metal/gen7.hpp"

namespace hakios::metal::gen7 {

void event(pm4::Stream& stream, a6xx::vgt_event_type e, uint64_t address, uint32_t value, bool write) {
    // freedreno_gpu_event.h's A7XX table. Fields shared with A6xx keep their
    // encodings; cache/CCU clean events have different event IDs.
    uint32_t code = e;
    switch (e) {
    case a6xx::CACHE_INVALIDATE: code = CACHE_INVALIDATE7; break;
    case a6xx::CACHE_FLUSH_TS: code = CACHE_CLEAN; break;
    case a6xx::PC_CCU_FLUSH_COLOR_TS: code = CCU_CLEAN_COLOR; break;
    case a6xx::PC_CCU_FLUSH_DEPTH_TS: code = CCU_CLEAN_DEPTH; break;
    case a6xx::PC_CCU_RESOLVE_TS: code = CCU_RESOLVE_CLEAN; break;
    default: break;
    }
    const uint32_t words[4] = {code | (write ? CP_EVENT_WRITE7_0_WRITE_ENABLED : 0u),
                                uint32_t(address), uint32_t(address >> 32), value};
    stream.packet(CP_EVENT_WRITE7, words, write ? 4 : 1); // CP_EVENT_WRITE7; USER_32B, RAM are both zero
}

void batchStart(pm4::Stream& stream) {
    const uint32_t thread = CP_SET_THREAD_BR | CP_THREAD_CONTROL_0_CONCURRENT_BIN_DISABLE; // CP_THREAD_CONTROL: BR, CONCURRENT_BIN_DISABLE
    stream.packet(CP_THREAD_CONTROL, &thread, 1);
    event(stream, a6xx::PC_CCU_INVALIDATE_COLOR);
    event(stream, a6xx::PC_CCU_INVALIDATE_DEPTH);
    const uint32_t unknown = 40; // tu6_init_hw(A7XX): UNK_40 (decimal 40)
    stream.packet(CP_EVENT_WRITE7, &unknown, 1);
    event(stream, a6xx::CACHE_INVALIDATE);
    stream.waitForIdle();
    const uint32_t invalidate = 0x01fffeff; // A7XX HLSQ: all stage, IBO, bindless state
    stream.registers(registerOffset(kChipA730, Register::HLSQ_INVALIDATE_CMD), &invalidate, 1);
    stream.waitForIdle();
    // A7xx moved the cache offsets out of RB_CCU_CNTL into CNTL2. Use full
    // sysmem caches: four 256 KiB depth caches, then four 64 KiB colour caches.
    // Concurrent resolve/unresolve stay disabled during the initial 2D ladder.
    const uint32_t ccu = 0;
    stream.registers(a6xx::REG_A6XX_RB_CCU_CNTL, &ccu, 1);
    const uint32_t colorOffset = kChipA730.numCcu * kChipA730.depthCacheBytesPerCcu;
    const uint32_t ccu2 = (colorOffset >> 12) << 23;
    stream.registers(registerOffset(kChipA730, Register::RB_CCU_CNTL2), &ccu2, 1);
    for (const auto& w : kMagic) stream.registers(w.reg, &w.value, 1);
    // Stale early preambles must not run from a previous owner's FS.
    const uint32_t zero = 0;
    stream.registers(a6xx::REG_A6XX_SP_FS_CTRL_REG0, &zero, 1);
}

void raw(pm4::Stream& stream, uint32_t a7xxOffset, uint32_t value) { stream.registers(a7xxOffset, &value, 1); }

namespace {
// fd6_emit_static_regs(A7XX) after its magic writes (which batchStart already made), in its order. The offsets are
// the A7XX ones as freedreno wrote them for FD730 under the drm-shim (device-info/.../metal-a730-draw/tri.txt);
// the magic-valued ones (RB/SP/TPL1/VPC/GRAS_DBG_ECO_CNTL, SP_CHICKEN_BITS, UCHE_*, RB_UNKNOWN_8E01, PC_MODE_CNTL)
// are batchStart's kMagic and not repeated here.
constexpr MagicWrite kDrawStatic[] = {
    {0xae04, 0x00000008},   // SP_FLOAT_CNTL: F16_NO_INF
    {0xae0f, 0x0000003f},   // SP_PERFCTR_ENABLE: VS HS DS GS FS CS
    {0xab20, 0},            // SP_IBO_COUNT
    {0xb182, 0},            // SP_UNKNOWN_B182
    {0xa9a8, 0},            // SP_UNKNOWN_A9A8
    {0xab00, 0x00000005},   // SP_MODE_CONTROL: CONSTANT_DEMOTION_ENABLE, ISAMMODE_GL
    {0xa009, 0x00000001},   // VFD_ADD_OFFSET: VERTEX
    {0x9107, 0},            // PC_RASTER_CNTL (A7XX's offset; on A6XX this is VPC_UNKNOWN_9107)
    {0x8811, 0x00000010},   // RB_UNKNOWN_8811
    {0x8101, 0},            // GRAS_LRZ_PS_INPUT_CNTL
    {0x8109, 0},            // GRAS_SAMPLE_CNTL
    {0x8110, 0x00000002},   // GRAS_UNKNOWN_8110
    {0x8818, 0},            // RB_UNKNOWN_8818 (8819..881E are A6XX only)
    {0x88f0, 0},            // RB_UNKNOWN_88F0
    {0x9236, 0},            // VPC_POINT_COORD_INVERT
    {0x9300, 0},            // VPC_UNKNOWN_9300
    {0x9306, 0x00000001},   // VPC_SO_DISABLE: DISABLE
    {0x9317, 0},            // PC_RASTER_CNTL_V2
    {0x9b07, 0},            // PC_MULTIVIEW_CNTL
    {0xb183, 0},            // SP_UNKNOWN_B183
    {0x8099, 0},            // GRAS_SU_CONSERVATIVE_RAS_CNTL
    {0x809b, 0},            // GRAS_VS_LAYER_CNTL
    {0x80a0, 0x00000002},   // GRAS_SC_CNTL: CCUSINGLECACHELINESIZE 2
    {0x80af, 0},            // GRAS_UNKNOWN_80AF
    {0x9602, 0},            // VPC_UNKNOWN_9602
    {0x9e72, 0},            // PC_UNKNOWN_9E72
    {0xb309, 0x000000a2},   // SP_TP_MODE_CNTL: ISAMMODE_GL | 0xa0
    {0xa9cb, 0x0000fcfc},   // HLSQ_CONTROL_5_REG: LINELENGTHREGID, FOVEATIONQUALITYREGID r63.x
    {0xa007, 0},            // VFD_MODE_CNTL: RENDERING_PASS
    {0xa008, 0},            // VFD_MULTIVIEW_CNTL
};
// after CP_SET_DRAW_STATE: the LRZ and depth-plane zeros (GRAS_LRZ_CNTL and RB_LRZ_CNTL twice, as Gallium does)
constexpr MagicWrite kDrawStaticAfterDrawState[] = {
    {0x9305, 0},   // VPC_SO_STREAM_CNTL
    {0x8100, 0},   // GRAS_LRZ_CNTL
    {0x810b, 0},   // GRAS_LRZ_CNTL2 (A7XX)
    {0x8898, 0},   // RB_LRZ_CNTL
    {0x8870, 0},   // RB_DEPTH_PLANE_CNTL
    {0x8094, 0},   // GRAS_SU_DEPTH_PLANE_CNTL
    {0x8100, 0},   // GRAS_LRZ_CNTL
    {0x8898, 0},   // RB_LRZ_CNTL
};
}  // namespace

void drawStatic(pm4::Stream& stream, uint64_t borderColors, uint64_t trace) {
    auto crumb = [&](uint32_t step) {
        if (trace != 0) stream.memWrite(trace + 4ull * step, &step, 1);
    };
    // RB_CCU_CNTL's static half as fd6_emit_static_regs(A7XX) writes it: GMEM_FAST_CLEAR_DISABLE, concurrent
    // resolve and unresolve disabled. It "requires a WFI to take effect" (fd6_emit.cc), so after one.
    stream.waitForIdle();
    raw(stream, a6xx::REG_A6XX_RB_CCU_CNTL, 0x00000001);
    crumb(13);
    for (const auto& w : kDrawStatic) raw(stream, w.reg, w.value);
    crumb(14);
    const uint32_t disableAll[3] = {0x00040000, 0, 0};   // CP_SET_DRAW_STATE: COUNT 0 | DISABLE_ALL_GROUPS | GROUP_ID 0
    stream.packet(a6xx::CP_SET_DRAW_STATE, disableAll, 3);
    crumb(15);
    for (const auto& w : kDrawStaticAfterDrawState) raw(stream, w.reg, w.value);
    for (uint32_t i = 0; i < 32; i++) raw(stream, 0xa012 + 4 * i, 0);   // VFD_FETCH[i].SIZE: no inherited BASE is fetched
    const uint32_t bc[2] = {uint32_t(borderColors), uint32_t(borderColors >> 32)};
    stream.registers(0xb302, bc, 2);   // SP_TP_BORDER_COLOR_BASE_ADDR
    stream.registers(0xb180, bc, 2);   // SP_PS_TP_BORDER_COLOR_BASE_ADDR
    raw(stream, 0x980a, 0);            // PC_DGEN_SU_CONSERVATIVE_RAS_CNTL
    const uint32_t bicubic[5] = {0, 0x3fe05ff4, 0x3fa0ebee, 0x3f5193ed, 0x3f0243f0};
    stream.registers(0xb608, bicubic, 5);   // TPL1_BICUBIC_WEIGHTS_TABLE_0..4 (CP_PROTECTed on A6XX)
    raw(stream, 0x9885, 0x00020000);   // PC_TESS_PARAM_SIZE (FD6_TESS_PARAM_SIZE)
    raw(stream, 0x9886, 0x00002000);   // PC_TESS_FACTOR_SIZE (FD6_TESS_FACTOR_SIZE)
    raw(stream, 0x8864, 0);            // RB_ALPHA_CONTROL (turnip's table; the A6XX path writes it too)
}

void sysmemPassBegin(pm4::Stream& stream, uint32_t W, uint32_t H) {
    // fd6_emit_sysmem_prep(A7XX) as freedreno wrote it: LRZ_FLUSH, the window scissor and the 2D resolve window, the
    // four window offsets 0, GRAS/RB_BIN_CONTROL 0 (A7XX has no BUFFERS_LOCATION field: RB_UNKNOWN_8812 = 0x3ff says
    // every buffer is in sysmem), RB_BIN_CONTROL2 0, the A7XX sysmem registers, the marker, the IB2 skips and the
    // visibility override. VPC_SO_DISABLE stays at the static 1 (no stream-out), as on the A6XX path.
    event(stream, a6xx::LRZ_FLUSH);
    const uint32_t window[2] = {0, ((H - 1) << 16) | (W - 1)};
    stream.registers(0x80f0, window, 2);   // GRAS_SC_WINDOW_SCISSOR_TL/BR
    stream.registers(0x840a, window, 2);   // GRAS_2D_RESOLVE_CNTL_1/2
    raw(stream, 0x8890, 0);                // RB_WINDOW_OFFSET
    raw(stream, 0x88d4, 0);                // RB_WINDOW_OFFSET2
    raw(stream, 0xab21, 0);                // SP_WINDOW_OFFSET
    raw(stream, 0xb307, 0);                // SP_TP_WINDOW_OFFSET
    raw(stream, 0x80a1, 0);                // GRAS_BIN_CONTROL: RENDERING_PASS, no bins
    raw(stream, 0x8800, 0);                // RB_BIN_CONTROL
    raw(stream, 0x88d3, 0);                // RB_BIN_CONTROL2
    raw(stream, 0x8812, 0x000003ff);       // RB_UNKNOWN_8812: all buffers in sysmem
    raw(stream, 0x8e06, 0x02080000);       // RB_UNKNOWN_8E06: the A730's magic value
    raw(stream, 0x8007, 0);                // GRAS_UNKNOWN_8007
    raw(stream, 0x8110, 0x00000002);       // GRAS_UNKNOWN_8110
    raw(stream, 0x8e09, 0x00000004);       // RB_UNKNOWN_8E09
    const uint32_t marker = 1;             // CP_SET_MARKER: RM6_DIRECT_RENDER
    stream.packet(a6xx::CP_SET_MARKER, &marker, 1);
    const uint32_t zero = 0, one = 1;
    stream.packet(a6xx::CP_SKIP_IB2_ENABLE_GLOBAL, &zero, 1);
    stream.packet(a6xx::CP_SET_VISIBILITY_OVERRIDE, &one, 1);
}

} // namespace hakios::metal::gen7
