#include "hakios-metal/gpu.hpp"
#include "hakios-metal/pm4.hpp"

#include <cerrno>
#include <cstring>
#include <iterator>
#include <unordered_map>

namespace hakios::metal {

// newBo's answers, for libhakios's profile (hakios_metal_bo_cache_stats): from the cache at its size, from the cache a
// larger one, new because the cache's were the GPU's still, new because the cache had none
std::atomic<uint64_t> gBoCacheStats[4];

namespace {
constexpr uint64_t kPage = 0x4000;  // Darwin's page; the driver rounds to its own (4 KB) inside it

MsmTimespec deadline(Transport& t, int64_t ns) {
    // the driver takes a deadline, not a length: a time of the clock that goes on while the machine sleeps
    // (mach_continuous_time, which is Darwin's CLOCK_MONOTONIC), as Linux's msm takes one of CLOCK_MONOTONIC
    if (ns < 0) {
        ns = 0;
    }
    int64_t at = t.nowNs() + ns;
    return MsmTimespec{at / 1000000000, at % 1000000000};
}
}  // namespace

int Gpu::param(uint32_t which, uint64_t* value) {
    MsmParam p{};
    p.pipe = kPipe3D;
    p.param = which;
    int e = t_->call(kGetParam, &p, sizeof(p));
    if (e == 0) {
        *value = p.value;
    }
    return e;
}

uint32_t Gpu::chipId() {
    // (asked once: every command buffer wants it, and each asking was a call into the kernel)
    uint32_t c = chip_.load(std::memory_order_relaxed);
    if (c == ~0u) {
        uint64_t v = 0;
        c = param(kParamChipId, &v) == 0 ? uint32_t(v) : 0;
        chip_.store(c, std::memory_order_relaxed);
    }
    return c;
}

uint64_t Gpu::faults() {
    uint64_t v = 0;
    return param(kParamFaults, &v) == 0 ? v : 0;
}

void Gpu::cacheTrim(int64_t now, uint64_t keep, std::vector<Cached>* gone) {
    for (auto it = cache_.begin(); it != cache_.end();) {
        auto& v = it->second;
        size_t old = 0;
        while (old < v.size() && now - v[old].freedNs > kCacheKeepNs) old++;
        for (size_t i = 0; i < old; i++) {
            cacheBytes_ -= v[i].size;
            gone->push_back(v[i]);
        }
        v.erase(v.begin(), v.begin() + ptrdiff_t(old));
        it = v.empty() ? cache_.erase(it) : std::next(it);
    }
    while (cacheBytes_ + keep > kCacheBytes && !cache_.empty()) {
        // (the oldest of all: the front of the vector whose front is oldest)
        auto oldest = cache_.begin();
        for (auto it = cache_.begin(); it != cache_.end(); ++it) {
            if (it->second.front().freedNs < oldest->second.front().freedNs) oldest = it;
        }
        cacheBytes_ -= oldest->second.front().size;
        gone->push_back(oldest->second.front());
        oldest->second.erase(oldest->second.begin());
        if (oldest->second.empty()) cache_.erase(oldest);
    }
}

void Gpu::closeCached(const std::vector<Cached>& gone) {
    for (const Cached& c : gone) {
        t_->unmap(c.handle, c.cpu);
        DrmGemClose close{c.handle, 0};
        t_->call(kGemClose, &close, sizeof(close));
    }
}

uint64_t Gpu::cachedBytes() {
    std::lock_guard<std::mutex> g(cacheLock_);
    return cacheBytes_;
}

std::unique_ptr<Bo> Gpu::newBo(uint64_t size, uint32_t flags, int* error, bool zeroed) {
    *error = 0;
    if (size == 0) {
        *error = EINVAL;
        return nullptr;
    }
    MsmGemNew n{};
    n.size = (size + kPage - 1) & ~(kPage - 1);
    n.flags = flags;
    {
        // one let go of before, of this size: given again once the GPU is done with it (the newest first: likeliest to
        // be done, and in the processor's cache). One the GPU is still at stays for later.
        std::vector<Cached> gone;
        std::unique_ptr<Bo> again;
        {
            // (of this size or up to a quarter larger, the smallest first, as freedreno's buckets round a size up: the
            // window server's offscreen textures follow the layers they draw, a few rows more or less every frame)
            std::lock_guard<std::mutex> g(cacheLock_);
            cacheTrim(t_->nowNs(), 0, &gone);
            bool busy = false;
            const uint64_t most = n.size + n.size / 4;
            for (auto it = cache_.lower_bound({n.size, 0}); it != cache_.end() && it->first.first <= most && again == nullptr;) {
                if (it->first.second != flags) {
                    ++it;
                    continue;
                }
                auto& v = it->second;
                for (size_t i = v.size(); i-- > 0 && again == nullptr;) {
                    MsmGemCpuPrep p{v[i].handle, kPrepRead | kPrepWrite | kPrepNoSync, MsmTimespec{}};
                    if (t_->call(kGemCpuPrep, &p, sizeof(p)) != 0) {
                        busy = true;
                        continue;
                    }
                    again.reset(new Bo(*this, v[i].handle, v[i].size, v[i].iova, v[i].cpu));
                    cacheBytes_ -= v[i].size;
                    v.erase(v.begin() + ptrdiff_t(i));
                }
                it = v.empty() ? cache_.erase(it) : std::next(it);
            }
            gBoCacheStats[again == nullptr ? (busy ? 2 : 3) : (again->size_ == n.size ? 0 : 1)]++;
        }
        closeCached(gone);
        if (again != nullptr) {
            if (zeroed) memset(again->cpu_, 0, again->size_);
            again->counted_ = true;
            again->flags_ = flags;
            allocated_ += again->size_;
            return again;
        }
    }
    if ((*error = t_->call(kGemNew, &n, sizeof(n))) != 0) {
        return nullptr;
    }
    MsmGemInfo info{};
    info.handle = n.handle;
    info.info = kInfoGetIova;
    if ((*error = t_->call(kGemInfo, &info, sizeof(info))) != 0) {
        DrmGemClose c{n.handle, 0};
        t_->call(kGemClose, &c, sizeof(c));
        return nullptr;
    }
    void* cpu = t_->map(n.handle, n.size);
    if (cpu == nullptr) {
        DrmGemClose c{n.handle, 0};
        t_->call(kGemClose, &c, sizeof(c));
        *error = ENOMEM;
        return nullptr;
    }
    std::unique_ptr<Bo> bo(new Bo(*this, n.handle, n.size, info.value, cpu));
    bo->counted_ = true;
    bo->flags_ = flags;
    allocated_ += n.size;
    return bo;
}

std::unique_ptr<Bo> Gpu::surfaceBo(uint32_t surfaceId, SurfaceInfo* out, int* error) {
    HakiosSurface s{};
    s.surface = surfaceId;
    if ((*error = t_->call(kHakiosSurface, &s, sizeof(s))) != 0) {
        return nullptr;
    }
    MsmGemInfo info{};
    info.handle = s.handle;
    info.info = kInfoGetIova;
    if ((*error = t_->call(kGemInfo, &info, sizeof(info))) != 0) {
        DrmGemClose c{s.handle, 0};
        t_->call(kGemClose, &c, sizeof(c));
        return nullptr;
    }
    *out = SurfaceInfo{s.size, s.offset, s.rowBytes, s.width, s.height, s.pixelFormat, s.planes};
    return std::unique_ptr<Bo>(new Bo(*this, s.handle, s.size, info.value, t_->map(s.handle, s.size)));
}

void Gpu::disown(Bo& bo) {
    if (bo.cpu_ != nullptr) {
        t_->unmap(bo.handle_, bo.cpu_);
        bo.cpu_ = nullptr;
    }
    bo.owned_ = false;
    if (bo.counted_) {
        allocated_ -= bo.size_;
        bo.counted_ = false;
    }
}

void Gpu::disown(std::unique_ptr<Bo> bo) {
    if (bo != nullptr) disown(*bo);
}

Bo::~Bo() { gpu_.release(*this); }

void Gpu::release(Bo& bo) {
    if (!bo.owned_) {
        return;
    }
    const bool ours = bo.counted_;   // (newBo's: not a surface's, nor one disowned)
    if (bo.counted_) {
        allocated_ -= bo.size_;
        bo.counted_ = false;
    }
    if (ours && bo.cpu_ != nullptr && bo.size_ <= kCacheLargest) {
        // kept for the next newBo of its size (cache_); what no longer fits goes instead, the oldest first
        std::vector<Cached> gone;
        {
            std::lock_guard<std::mutex> g(cacheLock_);
            const int64_t now = t_->nowNs();
            cacheTrim(now, bo.size_, &gone);
            cache_[{bo.size_, bo.flags_}].push_back(Cached{bo.handle_, bo.flags_, bo.size_, bo.iova_, bo.cpu_, now});
            cacheBytes_ += bo.size_;
        }
        closeCached(gone);
        return;
    }
    if (bo.cpu_ != nullptr) {
        t_->unmap(bo.handle_, bo.cpu_);
    }
    DrmGemClose c{bo.handle_, 0};
    t_->call(kGemClose, &c, sizeof(c));
}

Gpu::~Gpu() {
    {
        std::unique_lock<std::mutex> g(queuesLock_);
        queuesGone_.wait(g, [this] { return queues_ == 0; });
    }
    forgetSurfaceBuffers(this);
    std::vector<Cached> gone;
    for (auto& [key, v] : cache_) gone.insert(gone.end(), v.begin(), v.end());
    cache_.clear();
    cacheBytes_ = 0;
    closeCached(gone);
}

Gpu::QueueHold::QueueHold(Gpu& gpu) : gpu_(gpu) {
    std::lock_guard<std::mutex> g(gpu_.queuesLock_);
    gpu_.queues_++;
}

Gpu::QueueHold::~QueueHold() {
    // (told under the lock: once it is let go the Gpu may be destroyed, and nothing of it is touched after)
    std::lock_guard<std::mutex> g(gpu_.queuesLock_);
    if (--gpu_.queues_ == 0) gpu_.queuesGone_.notify_all();
}

int Gpu::newQueue(uint32_t priority, uint32_t* id) {
    MsmSubmitQueue q{0, priority, 0};
    int e = t_->call(kSubmitQueueNew, &q, sizeof(q));
    if (e == 0) {
        *id = q.id;
    }
    return e;
}

void Gpu::closeQueue(uint32_t id) {
    if (id != 0) {
        uint32_t q = id;
        t_->call(kSubmitQueueClose, &q, sizeof(q));
    }
}

int Gpu::submit(uint32_t queue, const Bo& commands, uint32_t offsetBytes, uint32_t words, const std::vector<BoUse>& uses,
                uint32_t* fence) {
    return submit(queue, commands, std::vector<std::pair<uint32_t, uint32_t>>{{offsetBytes, words}}, uses, fence);
}

int Gpu::submit(uint32_t queue, const Bo& commands, const std::vector<std::pair<uint32_t, uint32_t>>& runs, const std::vector<BoUse>& uses,
                uint32_t* fence) {
    // (each run is a CP_INDIRECT_BUFFER of its own in the kernel's ring, whose IB_SIZE holds 20 bits: a longer one would
    // run cut short, so it is refused here, as is a count the kernel would refuse)
    if (runs.empty() || runs.size() > kMaxSubmitCmds) {
        return EINVAL;
    }
    for (const auto& [offsetBytes, words] : runs) {
        if (words == 0 || words > pm4::kMaxIbWords || offsetBytes % 4 != 0 || offsetBytes + 4ull * words > commands.size()) {
            return EINVAL;
        }
    }
    // the command buffer is the first of the buffers; the others follow it once each, read or written
    // (each handle's place in BOS by a table: a window server frame names some 150 buffers in a few thousand uses, and
    // looking each use up along the list was a tenth of what a commit cost on the processor)
    std::vector<MsmSubmitBo> bos;
    std::unordered_map<uint32_t, size_t> at;
    bos.reserve(uses.size() / 4 + 8);
    at.reserve(uses.size() / 4 + 8);
    bos.push_back(MsmSubmitBo{kSubmitBoRead, commands.handle(), commands.gpuAddress()});
    at.emplace(commands.handle(), 0);
    for (const BoUse& u : uses) {
        if (u.bo == &commands) {
            continue;
        }
        const uint32_t flag = u.write ? kSubmitBoWrite : kSubmitBoRead;
        const auto [it, added] = at.emplace(u.bo->handle(), bos.size());
        if (added) {
            bos.push_back(MsmSubmitBo{flag, u.bo->handle(), u.bo->gpuAddress()});
        } else {
            bos[it->second].flags |= flag;
        }
    }
    std::vector<MsmSubmitCmd> cmds(runs.size());
    for (size_t i = 0; i < runs.size(); i++) {
        cmds[i].type = kSubmitCmdBuf;
        cmds[i].submit_idx = 0;
        cmds[i].submit_offset = runs[i].first;
        cmds[i].size = runs[i].second * 4;
    }
    MsmGemSubmit s{};
    s.flags = kPipe3D;
    s.nr_bos = uint32_t(bos.size());
    s.nr_cmds = uint32_t(cmds.size());
    s.bos = uint64_t(uintptr_t(bos.data()));
    s.cmds = uint64_t(uintptr_t(cmds.data()));
    s.fence_fd = -1;
    s.queueid = queue;
    int e = t_->call(kGemSubmit, &s, sizeof(s));
    if (e == 0) {
        *fence = s.fence;
    }
    return e;
}

int Gpu::wait(uint32_t queue, uint32_t fence, int64_t timeoutNs) {
    MsmWaitFence w{};
    w.fence = fence;
    w.timeout = deadline(*t_, timeoutNs);
    w.queueid = queue;
    return t_->call(kWaitFence, &w, sizeof(w));
}

int Gpu::cpuPrep(const Bo& bo, uint32_t op, int64_t timeoutNs) {
    MsmGemCpuPrep p{bo.handle(), op, deadline(*t_, timeoutNs)};
    return t_->call(kGemCpuPrep, &p, sizeof(p));
}

void Gpu::cpuFini(const Bo& bo) {
    MsmGemCpuFini f{bo.handle()};
    t_->call(kGemCpuFini, &f, sizeof(f));
}

}  // namespace hakios::metal
