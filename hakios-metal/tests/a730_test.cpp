// The A730 model validates commands/descriptors; it does not execute ir3 ISA.
// Mutation cases must fail independently of the encoder's success cases.
#include <cstring>
#include <map>
#include "hakios-metal/command.hpp"
#include "hakios-metal/fake_gpu.hpp"
#include "hakios-metal/gen7.hpp"
#include "hakios-metal/texture.hpp"
#include "fixtures/a730/end_only.hpp"
#include "fixtures/a730/fill_id.hpp"
#include "fixtures/a730/fill_id_air.hpp"
#include "fixtures/a730/tri_flat_fs_air.hpp"
#include "fixtures/a730/tri_flat_vs_air.hpp"
#include "fixtures/a730/tri_color_vs_air.hpp"
#include "fixtures/a730/tri_tex_fs_air.hpp"
#include "fixtures/a730/tri_dpos_vs_air.hpp"
#include "hakios-metal/render.hpp"
#include <cstdio>
#include <cstdlib>
#include "test.hpp"

using namespace hakios::metal;

namespace {
struct Rig {
    FakeGpu* fake;
    Gpu gpu;
    std::unique_ptr<CommandQueue> queue;
    Rig() : gpu([&] {
        auto f = std::make_unique<FakeGpu>(false, 0, true, kChipA730.id);
        fake = f.get(); return f;
    }()) {
        std::string why;
        queue = CommandQueue::make(gpu, &why);
        CHECK(queue != nullptr);
    }
    std::shared_ptr<Buffer> buffer(uint64_t bytes) {
        std::string why;
        auto b = newBuffer(gpu, bytes, &why);
        CHECK(b != nullptr); return b;
    }
};

void finish(const std::shared_ptr<CommandBuffer>& cb) {
    cb->commit(); cb->waitUntilCompleted();
    if (cb->status() != CommandBuffer::kCompleted) std::printf("       %s\n", cb->error().c_str());
    CHECK_EQ(cb->status(), CommandBuffer::kCompleted);
}
}

TEST(a730_queue_dispatches_and_reopens_without_losing_chip_state) {
    Rig r;
    const uint32_t groups[3] = {2, 1, 1};
    for (unsigned round = 0; round < 3; ++round) {
        auto cb = r.queue->commandBuffer();
        CHECK(cb->dispatch(kernels::a730_end_only, groups, nullptr, std::vector<Binding>{}));
        finish(cb);
        r.queue.reset();
        std::string why;
        r.queue = CommandQueue::make(r.gpu, &why);
        CHECK(r.queue != nullptr);
    }
    CHECK_EQ(r.fake->stats().dispatches, 3u);
    CHECK_EQ(r.fake->stats().faults, 0u);
}

TEST(a730_batch_start_matches_mesa_packet_order_and_key_registers) {
    pm4::Stream stream(kChipA730.id);
    gen7::batchStart(stream);
    const auto& words = stream.words();
    CHECK_EQ(words[0], pm4::type7(0x17, 1));
    CHECK_EQ(words[1], 0x08000001u);
    const uint32_t expectedEvents[] = {25, 24, 40, 51};
    for (unsigned i = 0; i < 4; ++i) {
        CHECK_EQ(words[2 + 2 * i], pm4::type7(0x46, 1));
        CHECK_EQ(words[3 + 2 * i], expectedEvents[i]);
    }
    bool cache = false, sp = false, preamble = false;
    for (size_t i = 0; i < words.size();) {
        const auto h = pm4::decode(words[i]);
        if (h.kind == pm4::Header::kRegisters) {
            if (h.reg == 0x0e17) cache = words[i + 1] == 0x00840004;
            if (h.reg == 0xae08) sp = words[i + 1] == 0x2400;
            if (h.reg == a6xx::REG_A6XX_SP_FS_CTRL_REG0) preamble = words[i + 1] == 0;
            CHECK(h.reg != 0x9107); // no A6xx VPC_UNKNOWN_9107 write over gen7 PC_RASTER_CNTL
        }
        i += h.count + 1;
    }
    CHECK(cache && sp && preamble);
    CHECK_EQ(sizeof(gen7::kMagic) / sizeof(gen7::kMagic[0]), 42u);
}

TEST(a730_cp_copy_and_2d_byte_copy_preserve_guards) {
    Rig r;
    auto src = r.buffer(512), dst = r.buffer(512);
    std::memset(src->contents(), 0x63, 512);
    std::memset(dst->contents(), 0xad, 512);
    auto cb = r.queue->commandBuffer();
    CHECK(cb->copy(*src, 4, *dst, 8, 64));
    CHECK(cb->copy(*src, 1, *dst, 101, 73));
    finish(cb);
    if (r.fake->stats().faults) std::printf("       fake: %s\n", r.fake->stats().lastFault.c_str());
    const auto* bytes = static_cast<const uint8_t*>(dst->contents());
    for (unsigned i = 0; i < 512; ++i) {
        CHECK_EQ(bytes[i], ((i >= 8 && i < 72) || (i >= 101 && i < 174)) ? 0x63 : 0xad);
    }
}

TEST(a730_air_dispatches_direct_and_indirect_with_four_core_private_memory) {
    Rig r;
    auto out = r.buffer(4096), constant = r.buffer(16), args = r.buffer(16);
    std::memset(constant->contents(), 0, 16);
    auto* counts = static_cast<uint32_t*>(args->contents());
    counts[0] = 2; counts[1] = 1; counts[2] = 1;
    Bindings bindings;
    bindings.buffers = {{out.get(), 0}, {constant.get(), 0}};
    for (const uint32_t y : {1u, 2u, 4u, 8u}) {
        auto cb = r.queue->commandBuffer();
        const uint32_t groups[3] = {2, 1, 1}, local[3] = {8, y, 1};
        CHECK(cb->dispatch(kernels::a730_fill_id_air, groups, local, bindings));
        CHECK(cb->dispatchIndirect(kernels::a730_fill_id_air, *args, 0, local, bindings));
        finish(cb);
        CHECK_EQ(r.fake->stats().lastDispatch.local[1], y);
    }
    PrivateMemory memory;
    std::string why;
    auto scratch = r.queue->privateMemory(512, false, &memory, &why);
    CHECK(scratch != nullptr);
    if (scratch) CHECK_EQ(scratch->length(), uint64_t(memory.perSp) * 4);
    CHECK_EQ(r.fake->stats().dispatches, 8u);
    CHECK_EQ(r.fake->stats().faults, 0u);
}

TEST(a730_2d_clear_and_texture_copy_use_gen7_offsets_and_events) {
    Rig r;
    TextureDesc desc;
    desc.width = 9; desc.height = 7;
    desc.format = PixelFormat::kRGBA8Unorm;
    desc.usage = kTextureUsageRenderTarget | kTextureUsageRead | kTextureUsageWrite;
    std::string why;
    auto src = Texture::make(r.gpu, desc, &why), dst = Texture::make(r.gpu, desc, &why);
    CHECK(src != nullptr && dst != nullptr);
    if (!src || !dst) return;
    RenderPass pass;
    pass.color0 = src.get(); pass.load = LoadAction::kClear;
    pass.clearColor[0] = 1; pass.clearColor[1] = 0; pass.clearColor[2] = 1; pass.clearColor[3] = 1;
    auto cb = r.queue->commandBuffer();
    CHECK(cb->clear(pass));
    CHECK(cb->blit(*src, 0, {0, 0, 9, 7}, *dst, 0, {0, 0, 9, 7}, false));
    finish(cb);
    if (r.fake->stats().faults) std::printf("       fake: %s\n", r.fake->stats().lastFault.c_str());
    uint8_t pixels[9 * 7 * 4]{};
    CHECK(dst->read(0, 0, 0, 9, 7, pixels, 9 * 4, &why));
    for (unsigned i = 0; i < sizeof(pixels); ++i) CHECK_EQ(pixels[i], i % 4 == 1 ? 0 : 255);
}

// The A730's IBOs are bindless set 0 (compute.cpp; GPU-PLAN.md, 10 November): the dispatch marks SP_CS_CONFIG
// BINDLESS_IBO, points SP_CS_BINDLESS_BASE[0] at its table with 64-byte descriptors and prefetches from set 0. The fake
// takes that and refuses the A6xx's non-bindless form, a cleared config bit and a base without its descriptor size.
TEST(a730_ibos_are_bindless_set_0_and_non_bindless_is_refused) {
    Rig r;
    auto out = r.buffer(4096), program = r.buffer(4 * kernels::a730_fill_id.numWords), table = r.buffer(compute::kTableBytes),
         commands = r.buffer(65536);
    std::memcpy(program->contents(), kernels::a730_fill_id.words, 4 * kernels::a730_fill_id.numWords);
    pm4::Stream stream(kChipA730.id);
    std::string why;
    const uint32_t groups[3] = {16, 1, 1}, local[3] = {64, 1, 1}, imm[4] = {0x1234, 0, 0, 0};
    CHECK(compute::encode(stream, kernels::a730_fill_id, groups, local, std::vector<Binding>{{out.get(), 0}}, imm, program->gpuAddress(),
                          table->gpuAddress(), static_cast<uint8_t*>(table->contents()), &why));
    auto submit = [&](const std::vector<uint32_t>& words) {
        std::memcpy(commands->contents(), words.data(), 4 * words.size());
        uint32_t fence = 0;
        const int error = r.gpu.submit(0, commands->bo(), 0, uint32_t(words.size()),
                                       {{&program->bo(), false}, {&table->bo(), true}, {&out->bo(), true}}, &fence);
        return error ? error : r.gpu.wait(0, fence, 1000000000);
    };
    CHECK_EQ(submit(stream.words()), 0);
    CHECK_EQ(r.fake->stats().lastDispatch.ibos, 1u);
    CHECK_EQ(r.fake->stats().lastDispatch.iboTable, table->gpuAddress() + compute::kTableIbosAt);
    const uint32_t config = legacyRegisterOffset(kChipA730, a6xx::REG_A6XX_SP_CS_CONFIG);
    // 0: the prefetch non-bindless, from the table's address (the A6xx's form); 1: SP_CS_CONFIG without BINDLESS_IBO;
    // 2: SP_CS_BINDLESS_BASE[0] without its 64-byte descriptor size
    for (unsigned mutation = 0; mutation < 3; ++mutation) {
        auto words = stream.words(); bool changed = false;
        for (size_t i = 0; i < words.size() && !changed;) {
            const auto h = pm4::decode(words[i]);
            if (h.kind == pm4::Header::kRegisters) {
                if (mutation == 1 && h.reg == config) { words[i + 1] &= ~a6xx::A6XX_SP_CS_CONFIG_BINDLESS_IBO; changed = true; }
                if (mutation == 2 && h.reg == 0xa9e8) { words[i + 1] &= ~3u; changed = true; }
            }
            if (mutation == 0 && h.kind == pm4::Header::kCommand && h.opcode == a6xx::CP_LOAD_STATE6_FRAG &&
                words[i + 1] == a6xx::loadState6_0(0, a6xx::ST6_IBO, a6xx::SS6_BINDLESS, a6xx::SB6_CS_SHADER, 1)) {
                const uint64_t from = table->gpuAddress() + compute::kTableIbosAt;
                words[i + 1] = a6xx::loadState6_0(0, a6xx::ST6_IBO, a6xx::SS6_INDIRECT, a6xx::SB6_CS_SHADER, 1);
                words[i + 2] = uint32_t(from); words[i + 3] = uint32_t(from >> 32);
                changed = true;
            }
            i += h.count + 1;
        }
        CHECK(changed);
        const int error = submit(words);
        if (!error) std::printf("       mutation %u was accepted\n", mutation);
        CHECK(error != 0);
    }
    CHECK_EQ(submit(stream.words()), 0);
}

TEST(a730_dispatch_mutations_fail_instead_of_validating_a6xx_state) {
    Rig r;
    auto program = r.buffer(4 * kernels::a730_end_only.numWords), table = r.buffer(compute::kTableBytes), commands = r.buffer(65536);
    std::memcpy(program->contents(), kernels::a730_end_only.words, 4 * kernels::a730_end_only.numWords);
    pm4::Stream stream(kChipA730.id);
    std::string why;
    const uint32_t groups[3] = {1, 1, 1}, local[3] = {1, 1, 1};
    Kernel kernel = kernels::a730_end_only;
    kernel.localSizeVariable = true;
    CHECK(compute::encode(stream, kernel, groups, local, std::vector<Binding>{}, nullptr, program->gpuAddress(), table->gpuAddress(),
                           static_cast<uint8_t*>(table->contents()), &why));
    auto submit = [&](const std::vector<uint32_t>& words) {
        std::memcpy(commands->contents(), words.data(), 4 * words.size());
        uint32_t fence = 0;
        const int error = r.gpu.submit(0, commands->bo(), 0, uint32_t(words.size()), {{&program->bo(), false}, {&table->bo(), true}}, &fence);
        return error ? error : r.gpu.wait(0, fence, 1000000000);
    };
    CHECK_EQ(submit(stream.words()), 0);
    // One mutation per fault: old HLSQ offset; wrong SP thread-size bit;
    // stale local size; corrupted magic state; missing write-enabled flag;
    // missing final cache clean; legacy event flags; wrong thread selection.
    for (unsigned mutation = 0; mutation < 11; ++mutation) {
        auto words = stream.words(); bool changed = false;
        for (size_t i = 0; i < words.size();) {
            const auto h = pm4::decode(words[i]);
            if (!changed && h.kind == pm4::Header::kRegisters) {
                if (mutation == 0 && h.reg == 0xa9cd) { words[i] = pm4::type4(0xb990, h.count); changed = true; }
                if (mutation == 1 && h.reg == 0xa9c3) { words[i + 1] ^= 0x200; changed = true; }
                if (mutation == 2 && h.reg == 0xa9df) { words[i + 1] ^= 4; changed = true; }
                if (mutation == 3 && h.reg == 0xae08) { words[i + 1] ^= 1; changed = true; }
                if (mutation == 8 && h.reg == 0x88e5) { words[i + 1] ^= 4; changed = true; }
                if (mutation == 9 && h.reg == 0xa9c5) { words[i] = pm4::type4(0xa9b2, h.count); changed = true; }
            }
            if (!changed && h.kind == pm4::Header::kCommand && h.opcode == 0x46 && h.count == 4) {
                if (mutation == 4) { words[i + 1] &= ~0x08000000u; changed = true; }
                if (mutation == 5 && (words[i + 1] & 0xff) == 0x31) { words[i + 1] = 0x0800003f; changed = true; }
                if (mutation == 6) { words[i + 1] |= 0x40000000; changed = true; }
            }
            if (!changed && mutation == 7 && h.kind == pm4::Header::kCommand && h.opcode == 0x17) {
                words[i + 1] = 2; changed = true;
            }
            if (!changed && mutation == 10 && h.kind == pm4::Header::kCommand && h.opcode == a6xx::CP_EXEC_CS) {
                const uint32_t preload[] = {
                    pm4::type7(a6xx::CP_LOAD_STATE6_FRAG, 3),
                    a6xx::loadState6_0(0, a6xx::ST6_SHADER, a6xx::SS6_INDIRECT, a6xx::SB6_CS_SHADER, 1),
                    uint32_t(program->gpuAddress()), uint32_t(program->gpuAddress() >> 32)};
                words.insert(words.begin() + i, std::begin(preload), std::end(preload));
                changed = true;
                break;
            }
            i += h.count + 1;
        }
        CHECK(changed);
        const int error = submit(words);
        if (!error) std::printf("       mutation %u was accepted\n", mutation);
        CHECK(error != 0);
    }
    CHECK_EQ(submit(stream.words()), 0); // recovery must not poison the next submission
}

// The first A730 draw (GPU-PLAN.md section 6 step 4): tri_flat into 64x64 RGBA8 in sysmem, encoded as one whole pass.
// The stream must carry the state freedreno writes for FD730 (device-info/2026-11-08-initial/metal-a730-draw/tri.txt,
// recorded under the drm-shim), name no A6XX-only register, and keep the A6XX's preload out. HAKIOS_A730_RD=FILE
// writes the stream, the programs and the table as an .rd for Mesa's cffdump to decode beside freedreno's.
namespace {
void writeRd(const char* path, const std::vector<std::pair<uint64_t, std::vector<uint32_t>>>& buffers, uint64_t streamAt, uint32_t streamWords) {
    FILE* f = std::fopen(path, "wb");
    if (f == nullptr) return;
    auto section = [&](uint32_t type, const void* data, uint32_t bytes) {
        std::fwrite(&type, 4, 1, f);
        std::fwrite(&bytes, 4, 1, f);
        std::fwrite(data, 1, bytes, f);
    };
    const uint64_t chip = kChipA730.id;
    section(14, &chip, 8);   // RD_CHIP_ID
    for (const auto& [at, words] : buffers) {
        const uint32_t addr[3] = {uint32_t(at), uint32_t(4 * words.size()), uint32_t(at >> 32)};
        section(3, addr, 12);                                    // RD_GPUADDR
        section(12, words.data(), uint32_t(4 * words.size()));   // RD_BUFFER_CONTENTS
    }
    const uint32_t cmd[3] = {uint32_t(streamAt), streamWords, uint32_t(streamAt >> 32)};
    section(6, cmd, 12);   // RD_CMDSTREAM_ADDR
    std::fclose(f);
}
// the value of the last type-4 write covering REG
uint32_t lastWrite(const std::vector<uint32_t>& w, uint32_t reg, bool* found = nullptr) {
    uint32_t v = ~0u;
    if (found) *found = false;
    for (size_t i = 0; i < w.size();) {
        const pm4::Header h = pm4::decode(w[i]);
        if (h.kind == pm4::Header::kRegisters && reg >= h.reg && reg < h.reg + h.count) {
            v = w[i + 1 + (reg - h.reg)];
            if (found) *found = true;
        }
        i += 1 + h.count;
    }
    return v;
}
bool hasPacket(const std::vector<uint32_t>& w, uint32_t opcode, uint32_t firstWord, uint32_t mask) {
    for (size_t i = 0; i < w.size();) {
        const pm4::Header h = pm4::decode(w[i]);
        if (h.kind == pm4::Header::kCommand && h.opcode == opcode && h.count != 0 && (w[i + 1] & mask) == firstWord) return true;
        i += 1 + h.count;
    }
    return false;
}
}  // namespace

TEST(a730_tri_flat_sysmem_draw_carries_freedreno_state) {
    Rig r;
    TextureDesc desc;
    desc.width = 64;
    desc.height = 64;
    desc.format = PixelFormat::kRGBA8Unorm;
    desc.usage = kTextureUsageRenderTarget | kTextureUsageRead;
    std::string why;
    auto target = Texture::make(r.gpu, desc, &why);
    CHECK(target != nullptr);
    if (!target) return;
    const Program p{kernels::a730_tri_flat_vs_air, kernels::a730_tri_flat_fs_air, Linkage{}, "a730_tri_flat"};
    auto vsProgram = r.buffer(4ull * p.vs.k.numWords), fsProgram = r.buffer(4ull * p.fs.k.numWords), table = r.buffer(render::kTableBytes);
    memcpy(vsProgram->contents(), p.vs.k.words, 4ull * p.vs.k.numWords);
    memcpy(fsProgram->contents(), p.fs.k.words, 4ull * p.fs.k.numWords);
    RenderPass pass;
    pass.color0 = target.get();
    Draw d;
    d.vertexCount = 3;
    d.viewport = Viewport{0, 0, 64, 64, 0, 1};
    pm4::Stream stream(kChipA730.id);
    const bool ok = render::encode(stream, p, pass, d, Bindings{}, Bindings{}, vsProgram->gpuAddress(), fsProgram->gpuAddress(), table->gpuAddress(),
                                   static_cast<uint8_t*>(table->contents()), &why, render::Options{});
    if (!ok) std::printf("       %s\n", why.c_str());
    CHECK(ok);
    if (!ok) return;
    const auto& w = stream.words();
    if (const char* out = std::getenv("HAKIOS_A730_RD")) {
        auto words = [](const Buffer& b) {
            std::vector<uint32_t> v(b.length() / 4);
            memcpy(v.data(), b.contents(), 4 * v.size());
            return v;
        };
        const uint64_t streamAt = 0x7f000000;
        writeRd(out, {{vsProgram->gpuAddress(), words(*vsProgram)}, {fsProgram->gpuAddress(), words(*fsProgram)}, {table->gpuAddress(), words(*table)}, {streamAt, w}},
                streamAt, uint32_t(w.size()));
    }
    // freedreno's values for FD730 (tri.txt): registers that differ from the A6XX path, or exist on the A7XX alone
    struct Want {
        uint32_t reg, value;
        const char* name;
    };
    const Want wants[] = {
        {0x8e07, 0x00000001, "RB_CCU_CNTL (GMEM_FAST_CLEAR_DISABLE)"},
        {0x88e5, 0x80000000, "RB_CCU_CNTL2 (colour offset 1 MiB)"},
        {0x9107, 0x00000000, "PC_RASTER_CNTL"},
        {0x9317, 0x00000000, "PC_RASTER_CNTL_V2"},
        {0x9885, 0x00020000, "PC_TESS_PARAM_SIZE"},
        {0x9886, 0x00002000, "PC_TESS_FACTOR_SIZE"},
        {0xb609, 0x3fe05ff4, "TPL1_BICUBIC_WEIGHTS_TABLE_1"},
        {0x80a1, 0x00000000, "GRAS_BIN_CONTROL"},
        {0x8800, 0x00000000, "RB_BIN_CONTROL"},
        {0x8812, 0x000003ff, "RB_UNKNOWN_8812"},
        {0x8e06, 0x02080000, "RB_UNKNOWN_8E06"},
        {0x8e09, 0x00000004, "RB_UNKNOWN_8E09"},
        {0x8801, 0x00000000, "RB_RENDER_CNTL"},
        {0x8116, 0x00000000, "GRAS_SU_RENDER_CNTL"},
        {0x810b, 0x00000000, "GRAS_LRZ_CNTL2"},
        {0x8113, 0x00000000, "GRAS_LRZ_DEPTH_BUFFER_INFO"},
        {0xa82d, 0x00000000, "SP_VS_VGPR_CONFIG"},
        {0xaa01, 0x00000000, "SP_FS_VGPR_CONFIG"},
        {0xa9aa, 0x00000000, "HLSQ_FS_UNKNOWN_A9AA"},
        {0xa99e, 0x01ff7fc8, "SP_FS_PREFETCH_CNTL"},
        {0xa9c7, 0x00000007, "HLSQ_CONTROL_1_REG"},
        {0xa9ae, 0x00000300, "HLSQ_UNKNOWN_A9AE"},
        {0xaa02, 0x00000000, "SP_PS_ALIASED_COMPONENTS_CONTROL"},
        {0x9104, 0x00ffffff, "VPC_VS_LAYER_CNTL"},
        {0x9307, 0x00000003, "VPC_POLYGON_MODE2"},
        {0x9109, 0x00000000, "VPC_PRIMITIVE_CNTL_0"},
        {0x9809, 0x00000003, "PC_POLYGON_MODE"},
        {0x8871, 0x00000020, "RB_DEPTH_CNTL (Z_CLAMP_ENABLE)"},
        {0x8870, 0x00000001, "RB_DEPTH_PLANE_CNTL (LATE_Z)"},
        {0x8094, 0x00000001, "GRAS_SU_DEPTH_PLANE_CNTL (LATE_Z)"},
    };
    for (const Want& x : wants) {
        bool found = false;
        const uint32_t v = lastWrite(w, x.reg, &found);
        if (!found || v != x.value) std::printf("       %s (0x%04x): %s 0x%08x, freedreno 0x%08x\n", x.name, x.reg, found ? "wrote" : "not written", v, x.value);
        CHECK(found && v == x.value);
    }
    // RB_MRT[0].BUF_INFO: RGBA8 (FMT6_8_8_8_8_UNORM), and LOSSLESSCOMPEN exactly when the target is compressed
    CHECK_EQ(lastWrite(w, 0x8822) & 0xffu, 0x30u);
    CHECK_EQ((lastWrite(w, 0x8822) & 0x800u) != 0, target->compressed());
    // GRAS_CL_CNTL keeps Metal's ZERO_GB_SCALE_Z (freedreno's GL has none) and adds the A7XX's Z_CLAMP_ENABLE
    CHECK_EQ(lastWrite(w, 0x8000) & a6xx::A6XX_GRAS_CL_CNTL_Z_CLAMP_ENABLE, a6xx::A6XX_GRAS_CL_CNTL_Z_CLAMP_ENABLE);
    // no A6XX-only register: HLSQ_UNKNOWN_BE00/01, TPL1_UNKNOWN_B605, the legacy shading rate, VPC_UNKNOWN_9210/11,
    // and the A6XX offsets of the HLSQ registers that moved (HLSQ_VS_CNTL 0xb800, HLSQ_FS_CNTL_0 0xb980)
    for (uint32_t reg : {0xbe00u, 0xbe01u, 0xb605u, 0x8a00u, 0x8a10u, 0x8a20u, 0x8a30u, 0x9210u, 0x9211u, 0xb800u, 0xb980u}) {
        bool found = false;
        lastWrite(w, reg, &found);
        if (found) std::printf("       wrote 0x%04x, which the A7XX does not have at that offset\n", reg);
        CHECK(!found);
    }
    // no instruction preload (ST6_SHADER into SB6_VS_SHADER / SB6_FS_SHADER): the A7XX fetches from OBJ_START
    const uint32_t stMask = a6xx::CP_LOAD_STATE6_0_STATE_TYPE__MASK | a6xx::CP_LOAD_STATE6_0_STATE_BLOCK__MASK;
    CHECK(!hasPacket(w, a6xx::CP_LOAD_STATE6_GEOM, a6xx::loadState6_0(0, a6xx::ST6_SHADER, a6xx::SS6_INDIRECT, a6xx::SB6_VS_SHADER, 0) & stMask, stMask));
    CHECK(!hasPacket(w, a6xx::CP_LOAD_STATE6_FRAG, a6xx::loadState6_0(0, a6xx::ST6_SHADER, a6xx::SS6_INDIRECT, a6xx::SB6_FS_SHADER, 0) & stMask, stMask));
    // and the draw itself, the 3-word auto-indexed form
    CHECK(hasPacket(w, a6xx::CP_DRAW_INDX_OFFSET, a6xx::drawIndxOffset0(a6xx::DI_PT_TRILIST, a6xx::DI_SRC_SEL_AUTO_INDEX, a6xx::IGNORE_VISIBILITY), ~0u));
}

// The A730 fake's draw validation: the probe's three tri_flat shapes (one, pair, a Clear then one) through CommandBuffer::draw
// run clean, three rounds with the queue reopened; then the whole-pass stream of render::encode, submitted by hand, is
// refused when damaged in each way the phone's first hangs were (an A6XX register, an A6XX preload, a VGPR_CONFIG not 0,
// a missing gen7 pass register, the A6XX batch thread) and accepted again afterwards.
TEST(a730_fake_validates_sysmem_draws_and_refuses_a6xx_state) {
    Rig r;
    TextureDesc desc;
    desc.width = 64;
    desc.height = 64;
    desc.format = PixelFormat::kRGBA8Unorm;
    desc.usage = kTextureUsageRenderTarget | kTextureUsageRead;
    std::string why;
    const Program p{kernels::a730_tri_flat_vs_air, kernels::a730_tri_flat_fs_air, Linkage{}, "a730_tri_flat"};
    for (const char* shape : {"one", "pair", "clear"}) {
        for (unsigned round = 0; round < 3; ++round) {
            auto t = Texture::make(r.gpu, desc, &why);
            CHECK(t != nullptr);
            if (!t) return;
            RenderPass pass;
            pass.color0 = t.get();
            if (std::strcmp(shape, "clear") == 0) {
                pass.load = LoadAction::kClear;
                pass.clearColor[3] = 1;
            }
            Draw d;
            d.vertexCount = std::strcmp(shape, "pair") == 0 ? 6 : 3;
            d.viewport = Viewport{0, 0, 64, 64, 0, 1};
            auto cb = r.queue->commandBuffer();
            const bool ok = cb->draw(p, pass, d, Bindings{}, Bindings{}, render::Options{});
            if (!ok) std::printf("       %s: %s\n", shape, cb->error().c_str());
            CHECK(ok);
            finish(cb);
            if (r.fake->stats().faults) std::printf("       fake: %s\n", r.fake->stats().lastFault.c_str());
            r.queue.reset();
            r.queue = CommandQueue::make(r.gpu, &why);
            CHECK(r.queue != nullptr);
            if (!r.queue) return;
        }
    }
    CHECK_EQ(r.fake->stats().draws, 9u);
    CHECK_EQ(r.fake->stats().faults, 0u);

    auto target = Texture::make(r.gpu, desc, &why);
    CHECK(target != nullptr);
    if (!target) return;
    auto vsProgram = r.buffer(4ull * p.vs.k.numWords), fsProgram = r.buffer(4ull * p.fs.k.numWords), table = r.buffer(render::kTableBytes),
         commands = r.buffer(262144);
    memcpy(vsProgram->contents(), p.vs.k.words, 4ull * p.vs.k.numWords);
    memcpy(fsProgram->contents(), p.fs.k.words, 4ull * p.fs.k.numWords);
    RenderPass pass;
    pass.color0 = target.get();
    Draw d;
    d.vertexCount = 3;
    d.viewport = Viewport{0, 0, 64, 64, 0, 1};
    pm4::Stream stream(kChipA730.id);
    CHECK(render::encode(stream, p, pass, d, Bindings{}, Bindings{}, vsProgram->gpuAddress(), fsProgram->gpuAddress(), table->gpuAddress(),
                         static_cast<uint8_t*>(table->contents()), &why, render::Options{}));
    auto submit = [&](const std::vector<uint32_t>& words) {
        CHECK(4 * words.size() <= commands->length());
        std::memcpy(commands->contents(), words.data(), 4 * words.size());
        uint32_t fence = 0;
        const int error = r.gpu.submit(0, commands->bo(), 0, uint32_t(words.size()),
                                       {{&vsProgram->bo(), false}, {&fsProgram->bo(), false}, {&table->bo(), true}, {&target->buffer()->bo(), true}}, &fence);
        return error ? error : r.gpu.wait(0, fence, 1000000000);
    };
    const uint32_t faults = r.fake->stats().faults;
    CHECK_EQ(submit(stream.words()), 0);
    CHECK_EQ(r.fake->stats().faults, faults);
    if (r.fake->stats().faults != faults) std::printf("       fake: %s\n", r.fake->stats().lastFault.c_str());
    // 0: HLSQ_FS_CNTL_0 at its A6XX offset 0xb980 (the A7XX's 0xab03); 1: SP_FS_VGPR_CONFIG 1; 2: RB_UNKNOWN_8812 not written
    // (renamed to an unused register); 3: an A6XX-style preload of the VS program before the draw; 4: CP_THREAD_CONTROL the
    // A6XX way (no BR selection); 5: PC_RASTER_CNTL at the A6XX's 0x9980
    const uint32_t fsCntl0 = legacyRegisterOffset(kChipA730, a6xx::REG_A6XX_HLSQ_FS_CNTL_0);
    for (unsigned mutation = 0; mutation < 6; ++mutation) {
        auto words = stream.words();
        bool changed = false;
        for (size_t i = 0; i < words.size() && !changed;) {
            const auto h = pm4::decode(words[i]);
            if (h.kind == pm4::Header::kRegisters && h.count == 1) {
                if (mutation == 0 && h.reg == fsCntl0) { words[i] = pm4::type4(0xb980, 1); changed = true; }
                if (mutation == 1 && h.reg == 0xaa01) { words[i + 1] = 1; changed = true; }
                if (mutation == 2 && h.reg == 0x8812) { words[i] = pm4::type4(0x8813, 1); changed = true; }
                if (mutation == 5 && h.reg == 0x9107) { words[i] = pm4::type4(0x9980, 1); changed = true; }
            }
            if (mutation == 3 && h.kind == pm4::Header::kCommand && h.opcode == a6xx::CP_DRAW_INDX_OFFSET) {
                const uint32_t preload[] = {pm4::type7(a6xx::CP_LOAD_STATE6_GEOM, 3),
                                            a6xx::loadState6_0(0, a6xx::ST6_SHADER, a6xx::SS6_INDIRECT, a6xx::SB6_VS_SHADER, 1),
                                            uint32_t(vsProgram->gpuAddress()), uint32_t(vsProgram->gpuAddress() >> 32)};
                words.insert(words.begin() + i, std::begin(preload), std::end(preload));
                changed = true;
                break;
            }
            if (mutation == 4 && h.kind == pm4::Header::kCommand && h.opcode == 0x17) {
                words[i + 1] = 2;
                changed = true;
            }
            i += h.count + 1;
        }
        CHECK(changed);
        const uint32_t before = r.fake->stats().faults;
        const int error = submit(words);
        if (!error) std::printf("       mutation %u was accepted\n", mutation);
        CHECK(error != 0);
        CHECK(r.fake->stats().faults > before);
        // each refused for its own reason
        const char* const reasons[] = {"wrote 0xb980", "SP_FS_VGPR_CONFIG", "RB_UNKNOWN_8812", "preload of the vertex", "select BR", "wrote 0x9980"};
        const bool named = r.fake->stats().lastFault.find(reasons[mutation]) != std::string::npos;
        if (!named) std::printf("       mutation %u: \"%s\", not \"%s\"\n", mutation, r.fake->stats().lastFault.c_str(), reasons[mutation]);
        CHECK(named);
    }
    const uint32_t drawsBefore = r.fake->stats().draws;
    CHECK_EQ(submit(stream.words()), 0);   // a fault leaves nothing behind for the next submission
    CHECK_EQ(r.fake->stats().draws, drawsBefore + 1);
}

// The probe's own submissions as the phone runs them, through CommandQueue (GPU-PLAN.md section 7, "Next" step 1): the
// fill_id_air control, then tri_flat --scissor-out --trace through CommandBuffer::draw. The draw's whole submission must be
// the A730's: the gen7 batch start (CP_THREAD_CONTROL), the trace's steps, no A6XX-only register. (Its pass streams
// were once built at the default chip, so render::encode's A730 path passed while CommandBuffer::draw sent the A6XX
// path to the phone and hung it.) HAKIOS_A730_SUBMIT_RD=PREFIX also writes PREFIX-air.rd and PREFIX-tri.rd, every buffer
// of each submission, for cffdump.
namespace {
class RecordingTransport : public Transport {
public:
    RecordingTransport(std::unique_ptr<FakeGpu> f) : inner_(std::move(f)) {}
    FakeGpu* fake() { return inner_.get(); }
    std::string path;   // the next submission's .rd ("" for none)
    std::vector<uint32_t> last;   // the last submission's command words
    int call(uint32_t request, void* data, size_t size) override {
        if (request == kGemSubmit) record(*static_cast<MsmGemSubmit*>(data));
        const int e = inner_->call(request, data, size);
        if (e == 0 && request == kGemNew) sizes_[static_cast<MsmGemNew*>(data)->handle] = static_cast<MsmGemNew*>(data)->size;
        if (e == 0 && request == kGemInfo && static_cast<MsmGemInfo*>(data)->info == kInfoGetIova)
            iovas_[static_cast<MsmGemInfo*>(data)->handle] = static_cast<MsmGemInfo*>(data)->value;
        return e;
    }
    void* map(uint32_t handle, uint64_t size) override { return inner_->map(handle, size); }
    void unmap(uint32_t handle, void* address) override { inner_->unmap(handle, address); }
    int64_t nowNs() override { return inner_->nowNs(); }

private:
    void record(const MsmGemSubmit& s) {
        auto* bos = reinterpret_cast<const MsmSubmitBo*>(uintptr_t(s.bos));
        auto* cmd = reinterpret_cast<const MsmSubmitCmd*>(uintptr_t(s.cmds));
        std::vector<std::pair<uint64_t, std::vector<uint32_t>>> buffers;
        for (uint32_t i = 0; i < s.nr_bos; i++) {
            const uint64_t at = iovas_[bos[i].handle], bytes = sizes_[bos[i].handle];
            std::vector<uint8_t> m = inner_->memory(at, bytes);
            std::vector<uint32_t> w(m.size() / 4);
            memcpy(w.data(), m.data(), 4 * w.size());
            if (i == cmd->submit_idx) last.assign(w.begin() + cmd->submit_offset / 4, w.begin() + (cmd->submit_offset + cmd->size) / 4);
            buffers.push_back({at, std::move(w)});
        }
        if (path.empty()) return;
        writeRd(path.c_str(), buffers, iovas_[bos[cmd->submit_idx].handle] + cmd->submit_offset, cmd->size / 4);
        std::printf("       wrote %s: %u buffers, %u command words\n", path.c_str(), s.nr_bos, cmd->size / 4);
        path.clear();
    }
    std::unique_ptr<FakeGpu> inner_;
    std::map<uint32_t, uint64_t> iovas_, sizes_;
};
}  // namespace

TEST(a730_probe_submissions_as_rd) {
    const char* env = std::getenv("HAKIOS_A730_SUBMIT_RD");
    const std::string prefix = env != nullptr ? env : "";
    RecordingTransport* rec = nullptr;
    Gpu gpu([&] {
        auto t = std::make_unique<RecordingTransport>(std::make_unique<FakeGpu>(false, 0, true, kChipA730.id));
        rec = t.get();
        return t;
    }());
    std::string why;
    {
        auto queue = CommandQueue::make(gpu, &why);
        CHECK(queue != nullptr);
        auto src = newBuffer(gpu, 8192, &why), dst = newBuffer(gpu, 8192, &why);
        static_cast<uint32_t*>(src->contents())[0] = 0x1234;
        auto cb = queue->commandBuffer();
        const uint32_t groups[3] = {16, 1, 1}, local[3] = {64, 1, 1};
        CHECK(cb->dispatch(kernels::a730_fill_id_air, groups, local, std::vector<Binding>{{dst.get(), 0}, {src.get(), 0}}));
        if (env) rec->path = prefix + "-air.rd";
        cb->commit();
        cb->waitUntilCompleted();
    }
    {
        auto queue = CommandQueue::make(gpu, &why);
        CHECK(queue != nullptr);
        TextureDesc desc;
        desc.format = PixelFormat::kRGBA8Unorm;
        desc.width = 64;
        desc.height = 64;
        desc.usage = kTextureUsageRenderTarget | kTextureUsageRead;
        auto t = Texture::make(gpu, desc, &why);
        CHECK(t != nullptr);
        if (!t) return;
        const Program flat{kernels::a730_tri_flat_vs_air, kernels::a730_tri_flat_fs_air, Linkage{}, "a730_tri_flat"};
        RenderPass pass;
        pass.color0 = t.get();
        Draw d;
        d.vertexStart = 0;
        d.vertexCount = 3;
        d.viewport = Viewport{0, 0, 64, 64, 0, 1};
        d.scissor = Scissor{63, 0, 1, 1};
        render::setA730Trace(t->gpuAddress() + 48 * 4);
        auto cb = queue->commandBuffer();
        const bool ok = cb->draw(flat, pass, d, Bindings{}, Bindings{}, render::Options{});
        if (!ok) std::printf("       %s\n", cb->error().c_str());
        CHECK(ok);
        if (env) rec->path = prefix + "-tri.rd";
        cb->commit();
        cb->waitUntilCompleted();
        CHECK_EQ(cb->status(), CommandBuffer::kCompleted);   // the A730 fake validates the draw too
        if (cb->status() != CommandBuffer::kCompleted) std::printf("       %s\n", cb->error().c_str());
        render::setA730Trace(0);
    }
    const std::vector<uint32_t>& w = rec->last;
    CHECK(!w.empty());
    if (w.empty()) return;
    // the gen7 batch start: CP_THREAD_CONTROL selecting BR, concurrent binning off
    CHECK(hasPacket(w, 0x17, 0x08000001u, ~0u));
    // every trace step of the batch start and the pass begin, as CP_MEM_WRITE of its own number
    for (uint32_t step : {uint32_t(render::kTraceEntry), uint32_t(render::kTraceSetMode), uint32_t(render::kTraceGen7Start), uint32_t(render::kTraceBatchStart),
                          uint32_t(render::kTracePassBegin)}) {
        bool found = false;
        for (size_t i = 0; i < w.size() && !found;) {
            const pm4::Header h = pm4::decode(w[i]);
            found = h.kind == pm4::Header::kCommand && h.opcode == pm4::kMemWrite && h.count == 3 && w[i + 3] == step;
            i += 1 + h.count;
        }
        if (!found) std::printf("       trace step %u not in the draw's submission\n", step);
        CHECK(found);
    }
    // no register the A7XX lacks or has moved (section 6's audit), anywhere in the submission
    for (uint32_t reg : {0xbb08u, 0xbb10u, 0xbb11u, 0xb605u, 0xbe00u, 0xbe01u, 0xbe04u, 0x9980u, 0x9981u, 0x9210u, 0x9211u, 0xb800u, 0xb801u, 0xb802u, 0xb803u,
                         0xb980u, 0xb982u, 0xb986u, 0xb4d1u, 0x8a00u, 0x8a10u, 0x8a20u, 0x8a30u}) {
        bool found = false;
        lastWrite(w, reg, &found);
        if (found) std::printf("       the draw's submission wrote 0x%04x, which the A7XX does not have at that offset\n", reg);
        CHECK(!found);
    }
}

// The FS texture prefetch on the A730 (fd6_program.cc emit_fs_inputs, A7XX): SP_FS_PREFETCH_CMD[i] in the A7XX's own field
// positions with its sampler and texture slots 0, the slots in SP_FS_BINDLESS_PREFETCH_CMD[i]. tri_tex (tri_color's VS,
// tri_tex_fs's one prefetch of texture 0 through sampler 0) draws clean through the fake, and its command decodes as the
// A7XX form; the A6XX form (the slots in the command) is refused.
TEST(a730_fs_prefetch_takes_the_a7xx_command_form) {
    Rig r;
    std::string why;
    TextureDesc td;
    td.width = 64;
    td.height = 64;
    td.format = PixelFormat::kRGBA8Unorm;
    td.usage = kTextureUsageRenderTarget | kTextureUsageRead;
    auto t = Texture::make(r.gpu, td, &why);
    td.width = td.height = 4;
    td.usage = kTextureUsageRead;
    auto tex = Texture::make(r.gpu, td, &why);
    CHECK(t != nullptr && tex != nullptr);
    if (!t || !tex) return;
    Sampler smp;
    CHECK(makeSampler(SamplerDesc{}, &smp, &why));
    const Program p{kernels::a730_tri_color_vs_air, kernels::a730_tri_tex_fs_air, Linkage{}, "a730_tri_tex"};
    CHECK_EQ(p.fs.numSamplerPrefetch, 1u);
    RenderPass pass;
    pass.color0 = t.get();
    Draw d;
    d.vertexCount = 6;
    d.viewport = Viewport{0, 0, 64, 64, 0, 1};
    Bindings fsb;
    fsb.textures = {tex.get()};
    fsb.samplers = {&smp};
    auto cb = r.queue->commandBuffer();
    const bool ok = cb->draw(p, pass, d, Bindings{}, fsb, render::Options{});
    if (!ok) std::printf("       %s\n", cb->error().c_str());
    CHECK(ok);
    finish(cb);
    if (r.fake->stats().faults) std::printf("       fake: %s\n", r.fake->stats().lastFault.c_str());
    CHECK_EQ(r.fake->stats().faults, 0u);
    CHECK_EQ(r.fake->stats().lastDraw.prefetchCount, 1u);
    const uint32_t cmd = r.fake->stats().lastDraw.prefetchCmd[0];
    const SamplerPrefetch& pf = p.fs.prefetch[0];
    CHECK_EQ(cmd, gen7::spFsPrefetchCmd(pf.src, pf.dst, pf.wrmask, pf.half != 0, pf.cmd));
    CHECK_EQ(cmd & (gen7::A7XX_SP_FS_PREFETCH_CMD_SAMP_ID__MASK | gen7::A7XX_SP_FS_PREFETCH_CMD_TEX_ID__MASK), 0u);

    // the same draw's stream with the A6XX command form: refused
    auto vsProgram = r.buffer(4ull * p.vs.k.numWords), fsProgram = r.buffer(4ull * p.fs.k.numWords), table = r.buffer(render::kTableBytes),
         commands = r.buffer(262144);
    memcpy(vsProgram->contents(), p.vs.k.words, 4ull * p.vs.k.numWords);
    memcpy(fsProgram->contents(), p.fs.k.words, 4ull * p.fs.k.numWords);
    pm4::Stream stream(kChipA730.id);
    CHECK(render::encode(stream, p, pass, d, Bindings{}, fsb, vsProgram->gpuAddress(), fsProgram->gpuAddress(), table->gpuAddress(),
                         static_cast<uint8_t*>(table->contents()), &why, render::Options{}));
    auto submit = [&](const std::vector<uint32_t>& words) {
        std::memcpy(commands->contents(), words.data(), 4 * words.size());
        uint32_t fence = 0;
        const int error = r.gpu.submit(0, commands->bo(), 0, uint32_t(words.size()),
                                       {{&vsProgram->bo(), false}, {&fsProgram->bo(), false}, {&table->bo(), true}, {&t->buffer()->bo(), true},
                                        {&tex->buffer()->bo(), false}},
                                       &fence);
        return error ? error : r.gpu.wait(0, fence, 1000000000);
    };
    CHECK_EQ(submit(stream.words()), 0);
    auto words = stream.words();
    bool changed = false;
    for (size_t i = 0; i < words.size() && !changed;) {
        const auto h = pm4::decode(words[i]);
        if (h.kind == pm4::Header::kRegisters && h.reg == a6xx::REG_A6XX_SP_FS_PREFETCH_CNTL && h.count == 2) {
            words[i + 2] = a6xx::spFsPrefetchCmd(pf.src, pf.sampId, pf.texId, pf.dst, pf.wrmask, pf.half != 0, pf.cmd);
            changed = true;
        }
        i += h.count + 1;
    }
    CHECK(changed);
    const uint32_t before = r.fake->stats().faults;
    CHECK(submit(words) != 0);
    CHECK(r.fake->stats().faults > before);
    CHECK(r.fake->stats().lastFault.find("SP_FS_PREFETCH_CMD") != std::string::npos);
    CHECK_EQ(submit(stream.words()), 0);
}

// Depth and stencil attachments on the A730 (tu6_emit_zs / fd6 emit_zs, A7XX): RB_DEPTH_BUFFER_INFO and RB_STENCIL_INFO name
// their tile mode, TILE6_3 (0x60 and 0xc); the rest is the A6XX's. A depth Clear, a draw under Less + write, a stencil Clear
// and a stencil test, and a pass that only clears the depth run clean through the fake; the planes without the tile mode
// are refused.
TEST(a730_depth_and_stencil_attachments_name_their_tile_mode) {
    Rig r;
    std::string why;
    TextureDesc cd;
    cd.width = cd.height = 64;
    cd.format = PixelFormat::kRGBA8Unorm;
    cd.usage = kTextureUsageRenderTarget | kTextureUsageRead;
    auto t = Texture::make(r.gpu, cd, &why);
    TextureDesc dd = cd;
    dd.format = PixelFormat::kDepth32FloatStencil8;
    dd.usage = kTextureUsageRenderTarget;
    auto ds = Texture::make(r.gpu, dd, &why);
    CHECK(t != nullptr && ds != nullptr);
    if (!t || !ds) return;
    const Program p{kernels::a730_tri_flat_vs_air, kernels::a730_tri_flat_fs_air, Linkage{}, "a730_tri_flat"};
    RenderPass pass;
    pass.color0 = t.get();
    pass.depth = ds.get();
    pass.depthLoad = LoadAction::kClear;
    pass.clearDepth = 1.0;
    pass.stencil = ds.get();
    pass.stencilLoad = LoadAction::kClear;
    pass.clearStencil = 0x5a;
    Draw d;
    d.vertexCount = 6;
    d.viewport = Viewport{0, 0, 64, 64, 0, 1};
    d.depth = DepthState{1, true};   // Less, write
    d.stencil.enabled = true;
    d.stencil.front = StencilFace{3, 0, 0, 2, 0xff, 0x0f};
    d.stencil.back = d.stencil.front;
    d.stencil.frontReference = d.stencil.backReference = 7;
    auto cb = r.queue->commandBuffer();
    const bool ok = cb->draw(p, pass, d, Bindings{}, Bindings{}, render::Options{});
    if (!ok) std::printf("       %s\n", cb->error().c_str());
    CHECK(ok);
    finish(cb);
    if (r.fake->stats().faults) std::printf("       fake: %s\n", r.fake->stats().lastFault.c_str());
    CHECK_EQ(r.fake->stats().faults, 0u);
    CHECK_EQ(r.fake->stats().lastDraw.depthBase, ds->gpuAddress());
    CHECK_EQ(r.fake->stats().tiledFills, 2u);   // the depth Clear and the stencil Clear
    // a pass that only clears the depth
    RenderPass depthOnly = pass;
    depthOnly.load = LoadAction::kLoad;
    depthOnly.stencil = nullptr;
    auto c = r.queue->commandBuffer();
    CHECK(c->clear(depthOnly));
    finish(c);
    CHECK_EQ(r.fake->stats().tiledFills, 3u);
    CHECK_EQ(r.fake->stats().faults, 0u);

    // the stream: the tile modes, and each one cleared is refused
    auto vsProgram = r.buffer(4ull * p.vs.k.numWords), fsProgram = r.buffer(4ull * p.fs.k.numWords), table = r.buffer(render::kTableBytes),
         commands = r.buffer(262144);
    memcpy(vsProgram->contents(), p.vs.k.words, 4ull * p.vs.k.numWords);
    memcpy(fsProgram->contents(), p.fs.k.words, 4ull * p.fs.k.numWords);
    pm4::Stream stream(kChipA730.id);
    CHECK(render::encode(stream, p, pass, d, Bindings{}, Bindings{}, vsProgram->gpuAddress(), fsProgram->gpuAddress(), table->gpuAddress(),
                         static_cast<uint8_t*>(table->contents()), &why, render::Options{}));
    CHECK_EQ(lastWrite(stream.words(), 0x8872), 0x64u);   // DEPTH6_32 | TILEMODE(TILE6_3)
    CHECK_EQ(lastWrite(stream.words(), 0x8881), 0x0du);   // SEPARATE_STENCIL | TILEMODE(TILE6_3)
    auto submit = [&](const std::vector<uint32_t>& words) {
        std::memcpy(commands->contents(), words.data(), 4 * words.size());
        uint32_t fence = 0;
        const int error = r.gpu.submit(0, commands->bo(), 0, uint32_t(words.size()),
                                       {{&vsProgram->bo(), false}, {&fsProgram->bo(), false}, {&table->bo(), true}, {&t->buffer()->bo(), true},
                                        {&ds->buffer()->bo(), true}},
                                       &fence);
        return error ? error : r.gpu.wait(0, fence, 1000000000);
    };
    CHECK_EQ(submit(stream.words()), 0);
    for (uint32_t which : {0x8872u, 0x8881u}) {
        auto words = stream.words();
        bool changed = false;
        for (size_t i = 0; i < words.size();) {
            const auto h = pm4::decode(words[i]);
            if (h.kind == pm4::Header::kRegisters && h.reg == which && h.count >= 1 && (words[i + 1] & (which == 0x8872 ? 0x60u : 0xcu)) != 0) {
                words[i + 1] &= which == 0x8872 ? ~0x60u : ~0xcu;
                changed = true;
            }
            i += h.count + 1;
        }
        CHECK(changed);
        const uint32_t before = r.fake->stats().faults;
        CHECK(submit(words) != 0);
        CHECK(r.fake->stats().faults > before);
        CHECK(r.fake->stats().lastFault.find("TILEMODE") != std::string::npos);
    }
    CHECK_EQ(submit(stream.words()), 0);
}

// Indirect draws and occlusion queries on the A730: CP_DRAW_INDIRECT_MULTI as on the A6XX (turnip's A7XX form is the same
// packet); the sample count by RB_SAMPLE_COUNT_ADDR and ZPASS_DONE, each followed by CCU_CLEAN_DEPTH (the A730 is A7XX
// gen1: no CP_EVENT_WRITE7 sample-count write). The fake reads the indirect arguments when the draw runs and counts the
// draw's samples into the query's word.
TEST(a730_indirect_draws_and_occlusion_queries) {
    Rig r;
    std::string why;
    TextureDesc cd;
    cd.width = cd.height = 64;
    cd.format = PixelFormat::kRGBA8Unorm;
    cd.usage = kTextureUsageRenderTarget | kTextureUsageRead;
    auto t = Texture::make(r.gpu, cd, &why);
    CHECK(t != nullptr);
    if (!t) return;
    const Program p{kernels::a730_tri_flat_vs_air, kernels::a730_tri_flat_fs_air, Linkage{}, "a730_tri_flat"};
    RenderPass pass;
    pass.color0 = t.get();
    auto args = r.buffer(64), vis = r.buffer(64);
    auto* w = static_cast<uint32_t*>(args->contents());
    w[1] = 3;
    w[2] = 1;
    w[3] = 0;
    w[4] = 0;
    Draw d;
    d.vertexCount = 3;
    d.viewport = Viewport{0, 0, 64, 64, 0, 1};
    Draw ind = d;
    ind.indirect = args.get();
    ind.indirectOffset = 4;
    auto cb = r.queue->commandBuffer();
    const bool ok = cb->draw(p, pass, ind, Bindings{}, Bindings{}, render::Options{});
    if (!ok) std::printf("       %s\n", cb->error().c_str());
    CHECK(ok);
    w[1] = 6;
    w[2] = 2;
    w[3] = 9;
    finish(cb);
    if (r.fake->stats().faults) std::printf("       fake: %s\n", r.fake->stats().lastFault.c_str());
    CHECK_EQ(r.fake->stats().lastDraw.numIndices, 6u);
    CHECK_EQ(r.fake->stats().lastDraw.numInstances, 2u);
    CHECK_EQ(r.fake->stats().lastDraw.vertexIndexOffset, 9u);
    auto* q = static_cast<uint64_t*>(vis->contents());
    for (int i = 0; i < 8; i++) q[i] = 0xdeadbeefcafef00dull;
    auto counted = [&](uint32_t vertices, bool reset) {
        Draw c = d;
        c.vertexCount = vertices;
        c.visibility = vis.get();
        c.visibilityMode = 2;
        c.visibilityOffset = 16;
        c.visibilityReset = reset;
        auto b = r.queue->commandBuffer();
        CHECK(b->draw(p, pass, c, Bindings{}, Bindings{}, render::Options{}));
        finish(b);
        if (r.fake->stats().faults) std::printf("       fake: %s\n", r.fake->stats().lastFault.c_str());
        return q[2];
    };
    CHECK_EQ(counted(6, true), uint64_t(6));
    CHECK_EQ(counted(3, false), uint64_t(9));
    CHECK_EQ(q[1], 0xdeadbeefcafef00dull);
    CHECK_EQ(q[3], 0xdeadbeefcafef00dull);
    CHECK_EQ(r.fake->stats().faults, 0u);
}

// A device buffer in the vertex stage on the A730 (tri_dpos: the pair's positions by ldib through IBO 0): the compiler
// made the IBO bindless set 0's descriptor 0 (`.base0`), and the draw points SP_BINDLESS_BASE[0] (0xab0a) at its 3D IBO
// table with 64-byte descriptors, marks SP_VS_CONFIG BINDLESS_IBO, invalidates GFX_BINDLESS and prefetches SB6_IBO from
// set 0 (fd6_build_bindless_state<A7XX>). The fake takes that, and refuses the A6XX's SP_IBO load from memory.
TEST(a730_vertex_stage_ibos_are_bindless_set_0) {
    Rig r;
    std::string why;
    TextureDesc cd;
    cd.width = cd.height = 64;
    cd.format = PixelFormat::kRGBA8Unorm;
    cd.usage = kTextureUsageRenderTarget | kTextureUsageRead;
    auto t = Texture::make(r.gpu, cd, &why);
    CHECK(t != nullptr);
    if (!t) return;
    const float xy[6][2] = {{-1, -1}, {1, -1}, {-1, 1}, {1, -1}, {1, 1}, {-1, 1}};
    auto pos = r.buffer(sizeof xy);
    memcpy(pos->contents(), xy, sizeof xy);
    const Program p{kernels::a730_tri_dpos_vs_air, kernels::a730_tri_flat_fs_air, Linkage{}, "a730_tri_dpos"};
    CHECK_EQ(p.vs.k.numSsbos, 1u);
    RenderPass pass;
    pass.color0 = t.get();
    Draw d;
    d.vertexCount = 6;
    d.viewport = Viewport{0, 0, 64, 64, 0, 1};
    Bindings vsb;
    vsb.buffers = {Binding{pos.get(), 0}};
    auto cb = r.queue->commandBuffer();
    const bool ok = cb->draw(p, pass, d, vsb, Bindings{}, render::Options{});
    if (!ok) std::printf("       %s\n", cb->error().c_str());
    CHECK(ok);
    finish(cb);
    if (r.fake->stats().faults) std::printf("       fake: %s\n", r.fake->stats().lastFault.c_str());
    CHECK_EQ(r.fake->stats().faults, 0u);
    CHECK_EQ(r.fake->stats().lastDraw.gfxIbos, 1u);

    auto vsProgram = r.buffer(4ull * p.vs.k.numWords), fsProgram = r.buffer(4ull * p.fs.k.numWords), table = r.buffer(render::kTableBytes),
         commands = r.buffer(262144);
    memcpy(vsProgram->contents(), p.vs.k.words, 4ull * p.vs.k.numWords);
    memcpy(fsProgram->contents(), p.fs.k.words, 4ull * p.fs.k.numWords);
    pm4::Stream stream(kChipA730.id);
    CHECK(render::encode(stream, p, pass, d, vsb, Bindings{}, vsProgram->gpuAddress(), fsProgram->gpuAddress(), table->gpuAddress(),
                         static_cast<uint8_t*>(table->contents()), &why, render::Options{}));
    const auto& w = stream.words();
    CHECK_EQ(lastWrite(w, gen7::REG_A7XX_SP_BINDLESS_BASE), uint32_t(table->gpuAddress() + render::kTableVsAt) | 3u);
    CHECK((lastWrite(w, legacyRegisterOffset(kChipA730, a6xx::REG_A6XX_SP_VS_CONFIG)) & a6xx::A6XX_SP_VS_CONFIG_BINDLESS_IBO) != 0);
    CHECK((lastWrite(w, legacyRegisterOffset(kChipA730, a6xx::REG_A6XX_HLSQ_INVALIDATE_CMD)) & gen7::A7XX_HLSQ_INVALIDATE_CMD_GFX_BINDLESS__MASK) ==
          gen7::A7XX_HLSQ_INVALIDATE_CMD_GFX_BINDLESS__MASK);
    CHECK(hasPacket(w, a6xx::CP_LOAD_STATE6, a6xx::loadState6_0(0, a6xx::ST6_SHADER, a6xx::SS6_BINDLESS, a6xx::SB6_IBO, 1), ~0u));
    auto submit = [&](const std::vector<uint32_t>& words) {
        std::memcpy(commands->contents(), words.data(), 4 * words.size());
        uint32_t fence = 0;
        const int error = r.gpu.submit(0, commands->bo(), 0, uint32_t(words.size()),
                                       {{&vsProgram->bo(), false}, {&fsProgram->bo(), false}, {&table->bo(), true}, {&t->buffer()->bo(), true},
                                        {&pos->bo(), false}},
                                       &fence);
        return error ? error : r.gpu.wait(0, fence, 1000000000);
    };
    CHECK_EQ(submit(w), 0);
    // the A6XX's load (SS6_INDIRECT from the table) in its place: refused
    auto bad = w;
    bool changed = false;
    for (size_t i = 0; i < bad.size() && !changed;) {
        const auto h = pm4::decode(bad[i]);
        if (h.kind == pm4::Header::kCommand && h.opcode == a6xx::CP_LOAD_STATE6 && bad[i + 1] == a6xx::loadState6_0(0, a6xx::ST6_SHADER, a6xx::SS6_BINDLESS, a6xx::SB6_IBO, 1)) {
            const uint64_t from = table->gpuAddress() + render::kTableVsAt;
            bad[i + 1] = a6xx::loadState6_0(0, a6xx::ST6_SHADER, a6xx::SS6_INDIRECT, a6xx::SB6_IBO, 1);
            bad[i + 2] = uint32_t(from);
            bad[i + 3] = uint32_t(from >> 32);
            changed = true;
        }
        i += h.count + 1;
    }
    CHECK(changed);
    const uint32_t before = r.fake->stats().faults;
    CHECK(submit(bad) != 0);
    CHECK(r.fake->stats().faults > before);
    CHECK(r.fake->stats().lastFault.find("SB6_IBO") != std::string::npos);
    CHECK_EQ(submit(w), 0);
}
