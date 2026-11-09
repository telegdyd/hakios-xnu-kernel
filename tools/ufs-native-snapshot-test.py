#!/usr/bin/env python3
"""Test the actual immutable GPT snapshot reader, without firmware or storage access."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
source = (root / 'drivers/storage/hakios.ufs.c').read_text()
start = source.index('static int\nufs_gpt_snapshot(')
function = source[start:source.index('static bool\nufs_native_map(', start)]
harness = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uintptr_t vm_offset_t;
#define UFS_PAGE 0x4000UL
#define VM_WIMG_WCOMB 0
static struct { struct { uint64_t gpt_pa; uint32_t gpt_version,gpt_mask; } native; } page,*ufs_page=&page;
static uint8_t memory[49152] __attribute__((aligned(16384)));
static bool managed,fail_map;
static unsigned mappings;
static bool pmap_valid_address(uint64_t pa) { (void)pa;return managed; }
static vm_offset_t ml_io_map_unmappable(vm_offset_t pa,size_t n,unsigned flags) {
 assert(pa==(uintptr_t)memory && n==sizeof(memory) && !flags);mappings++;
 return fail_map?0:pa;
}
static uint32_t crc32(uint32_t c,const uint8_t *p,size_t n) {
 c=~c;while(n--) { c^=*p++;for(unsigned i=0;i<8;i++)c=(c>>1)^(0xedb88320u & (0u-(c&1))); }return ~c;
}
FUNCTION
int main(void) {
 uint8_t out[4096],*header=memory+8192+7*4096;uint32_t length=92,crc;
 memcpy(header,"EFI PART",8);memcpy(header+12,&length,4);crc=crc32(0,header,length);memcpy(header+16,&crc,4);
 page.native.gpt_pa=(uintptr_t)memory+8192;page.native.gpt_version=1;page.native.gpt_mask=1u<<7;
 fail_map=true;assert(ufs_gpt_snapshot(7,out)==ENOMEM && mappings==1);
 fail_map=false;assert(ufs_gpt_snapshot(7,out)==0 && !memcmp(out,header,4096) && mappings==2);
 assert(ufs_gpt_snapshot(7,out)==0 && mappings==2); /* one mapping, immutable snapshots */
 assert(ufs_gpt_snapshot(0,out)==ENXIO);assert(ufs_gpt_snapshot(8,out)==ENXIO);
 assert(ufs_gpt_snapshot(UINT32_MAX,out)==ENXIO);
 page.native.gpt_version=2;assert(ufs_gpt_snapshot(7,out)==ENXIO);page.native.gpt_version=1;
 page.native.gpt_pa++;assert(ufs_gpt_snapshot(7,out)==ENXIO);page.native.gpt_pa--;
 managed=true;assert(ufs_gpt_snapshot(7,out)==ENXIO);managed=false;
 uint64_t pa=page.native.gpt_pa;page.native.gpt_pa=UINT64_MAX & ~8191ULL;
 assert(ufs_gpt_snapshot(7,out)==ENXIO);page.native.gpt_pa=pa;
 header[40]^=1;assert(ufs_gpt_snapshot(7,out)==EINVAL);header[40]^=1;
 length=91;memcpy(header+12,&length,4);assert(ufs_gpt_snapshot(7,out)==EINVAL);
 length=4097;memcpy(header+12,&length,4);assert(ufs_gpt_snapshot(7,out)==EINVAL);
 length=92;memcpy(header+12,&length,4);header[0]=0;assert(ufs_gpt_snapshot(7,out)==EINVAL);
 puts("native UFS GPT snapshots: 8-KiB firmware alignment, mapping bounds, version/mask/range and CRC guards passed");
}
'''.replace('FUNCTION', function)
with tempfile.TemporaryDirectory() as temp:
    path = Path(temp)
    (path / 'test.c').write_text(harness)
    subprocess.run(['clang', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                    str(path / 'test.c'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)
