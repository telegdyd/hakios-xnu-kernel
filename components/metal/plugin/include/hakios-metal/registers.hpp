// One name per logical register, with offsets for both chips. Encoders for
// generation-specific fields and command ordering belong to their emitters.
#pragma once

#include "hakios-metal/chip.hpp"

namespace hakios::metal {

inline constexpr uint32_t kRegisterAbsent = 0xffffffffu;
enum class Register {
#define GPU_REG(name, a6, a7, fields) name,
#include "hakios-metal/chip-registers.inc"
#undef GPU_REG
    Count
};

struct RegisterInfo {
    uint32_t a6xx, a7xx;
    bool generationFields;
};
inline constexpr RegisterInfo kRegisters[] = {
#define GPU_REG(name, a6, a7, fields) {a6, a7, fields},
#include "hakios-metal/chip-registers.inc"
#undef GPU_REG
};

inline constexpr uint32_t registerOffset(const ChipInfo& chip, Register reg) {
    const auto i = static_cast<uint32_t>(reg);
    return i >= static_cast<uint32_t>(Register::Count) ? kRegisterAbsent :
           chip.generation == 6 ? kRegisters[i].a6xx :
           chip.generation == 7 ? kRegisters[i].a7xx : kRegisterAbsent;
}

// For shared emitters naming legacy registers. Only named table entries map;
// absent/unknown addresses are refused instead of falling through to A6xx.
inline constexpr uint32_t legacyRegisterOffset(const ChipInfo& chip, uint32_t a6xx) {
    if (chip.generation == 6) return a6xx;
    for (const auto& r : kRegisters) {
        if (r.a6xx == a6xx && a6xx != kRegisterAbsent) return r.a7xx;
    }
    return kRegisterAbsent;
}

}  // namespace hakios::metal
