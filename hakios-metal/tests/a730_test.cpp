// The A730 model validates commands/descriptors; it does not execute ir3 ISA.
// Mutation cases must fail independently of the encoder's success cases.
#include <cstring>
#include "hakios-metal/command.hpp"
#include "hakios-metal/fake_gpu.hpp"
#include "hakios-metal/gen7.hpp"
#include "hakios-metal/texture.hpp"
#include "fixtures/a730/end_only.hpp"
#include "fixtures/a730/fill_id.hpp"
#include "fixtures/a730/fill_id_air.hpp"
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
