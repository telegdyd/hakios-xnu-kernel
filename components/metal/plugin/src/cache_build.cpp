// hakios-metal: what the compiler is, for the compiled-stage cache's key (src/compiler.cpp: stageCacheName).
//
// SPDX-License-Identifier: MIT
//
// The Makefile's hash (HAKIOS_METAL_CACHE_BUILD) of the objects that make a stage -- compiler.o, link.o
// (render::link) and texture.o (makeSampler: the inline samplers' words) -- and of the Mesa archives linked in. The
// objects, not the sources: a stage is the code those objects run, so an edit of a header the compiler does not use
// (the encoder's, render.hpp's GMEM plan, gen7.hpp) or a comment leaves the key and the image's seeded stages as they
// were, while a change of Stage's or Sampler's layout, which the cache writes as it lies in memory, changes the
// compiler's code and so the key. Before, the key hashed compiler.cpp and every header: the A730 GMEM path's
// encoder headers emptied all 312 seeded stages. This file is its own object so that compiler.o does not hold the hash of itself.
// Without the define (the host tests, a build without Mesa), the old key: a build is its own.

namespace hakios::metal {

extern const char* const kCacheBuild;
#ifdef HAKIOS_METAL_CACHE_BUILD
const char* const kCacheBuild = HAKIOS_METAL_CACHE_BUILD;
#else
const char* const kCacheBuild = __DATE__ " " __TIME__;
#endif

}  // namespace hakios::metal
