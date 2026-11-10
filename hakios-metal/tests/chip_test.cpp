#include <cerrno>
#include <memory>

#include "hakios-metal/chip.hpp"
#include "hakios-metal/registers.hpp"
#include "hakios-metal/hakios_metal.h"
#include "hakios-metal/fake_gpu.hpp"
#include "test.hpp"

using namespace hakios::metal;

TEST(chip_identity_and_memory_match_each_kernel_target) {
    for (const auto* chip : {&kChipA635, &kChipA730}) {
        Gpu gpu(std::make_unique<FakeGpu>(false, 0, true, chip->id));
        CHECK_EQ(gpu.chipId(), chip->id);
        uint64_t value = 0;
        CHECK_EQ(gpu.param(kParamGmemSize, &value), 0);
        CHECK_EQ(value, chip->gmemBytes);
        CHECK_EQ(gpu.param(kParamHighestBankBit, &value), 0);
        CHECK_EQ(value, chip->highestBankBit);
        int error = 0;
        auto bo = gpu.newBo(4096, 0, &error);
        CHECK(bo != nullptr);
        CHECK_EQ(error, 0);
    }
    CHECK(chipInfo(0) == &kChipA635);
    CHECK(chipInfo(0x07040001) == nullptr);
    CHECK(hakios_metal_chip_known(kChipA730.id));
}

TEST(chip_a730_cache_split_and_tile_alignment) {
    CHECK_EQ(kChipA730.numCcu * kChipA730.depthCacheBytesPerCcu, 1024u * 1024);
    CHECK_EQ(kChipA730.gmemBytes - kChipA730.numCcu * kChipA730.colorCacheBytesPerCcu /
             kChipA730.gmemColorCacheDivisor, 0x1f8000u);
    CHECK_EQ(kChipA730.tileAlignW, 64u);
    CHECK_EQ(kChipA730.tileAlignH, 32u);
    CHECK(kChipA730.earlyPreamble);
}

TEST(chip_registers_distinguish_moved_absent_and_changed_fields) {
    CHECK_EQ(registerOffset(kChipA635, Register::PC_RASTER_CNTL), 0x9980u);
    CHECK_EQ(registerOffset(kChipA730, Register::PC_RASTER_CNTL), 0x9107u);
    // On A730, the old VPC offset would overwrite PC_RASTER_CNTL.
    CHECK_EQ(registerOffset(kChipA730, Register::VPC_UNKNOWN_9107), kRegisterAbsent);
    CHECK_EQ(registerOffset(kChipA730, Register::HLSQ_CS_CNTL_0), kRegisterAbsent);
    CHECK_EQ(registerOffset(kChipA635, Register::SP_CS_VGPR_CONFIG), kRegisterAbsent);
    CHECK_EQ(registerOffset(kChipA730, Register::SP_CS_VGPR_CONFIG), 0xa9c5u);
    CHECK_EQ(registerOffset(kChipA730, Register::SP_CS_CNTL_1), 0xa9c3u);
    CHECK_EQ(registerOffset(kChipA635, Register::SP_CS_CNTL_1), 0xa9c3u);
    CHECK(kRegisters[static_cast<unsigned>(Register::SP_CS_CNTL_1)].generationFields);
    CHECK(kRegisters[static_cast<unsigned>(Register::RB_2D_DST_INFO)].generationFields);
    CHECK_EQ(registerOffset(kChipA730, Register::Count), kRegisterAbsent);
}
