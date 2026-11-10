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

} // namespace hakios::metal::gen7
