#!/usr/bin/env python3
"""Test the actual native write policy/backend against a fake controller, never a phone."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
source = (root / 'drivers/storage/hakios.ufshc.c').read_text()
start = source.index('int\nufshc_allow_writes(')
end = source.index('/* ---- the proof, and the switches ---- */', start)
backend = source[start:end]
harness = r'''
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include "hakios.ufshc-write.h"
#define BLOCK 4096u
#define UCD_RESPONSE 512u
#define UTRD_CT_UFS (1u<<28)
#define UTRD_DD_OUT (1u<<25)
#define UTRD_DD_NONE 0u
struct hc_cmd { uint8_t upiu[32]; uint32_t dw0; };
struct hc_slot { uint32_t number; uint8_t *data; uint32_t *ucd; };
static struct ufshc_write_window hc_write_window;
static bool hc_usable, hc_in, hc_dead, hc_closed;
static unsigned hc_slot_buffer=8192, placements, rings, completions, gives, cleans;
static int hc_write_lock, depth, fail_place, fail_ring, fail_complete, residual;
static bool no_slot;
static uint8_t data[8192];
static uint32_t ucd[256];
static struct hc_slot slot={1,data,ucd};
static struct hc_cmd commands[16];
static void lck_mtx_lock(int *l) { (void)l; assert(depth++==0); }
static void lck_mtx_unlock(int *l) { (void)l; assert(--depth==0); }
static struct hc_slot *hc_take(bool wait) { assert(wait); return no_slot ? NULL : &slot; }
static void hc_give(struct hc_slot *s) { assert(s==&slot); gives++; }
static void hc_off(void) { hc_dead=true; }
static void FlushPoC_DcacheRegion(uintptr_t va, size_t n) {
 assert(va==(uintptr_t)data && n && n<=sizeof(data)); cleans++;
}
#define vm_offset_t uintptr_t
static int hc_place(struct hc_slot *s,const struct hc_cmd *c,uint32_t bytes) {
 assert(s==&slot && depth==1 && placements<16);
 assert(ufshc_write_permitted(&hc_write_window,c->upiu,c->dw0));
 assert(bytes<=sizeof(data));
 if(bytes) { assert(cleans==placements+1); for(unsigned i=0;i<bytes;i++)assert(data[i]==0xa5); }
 commands[placements++]=*c; return fail_place;
}
static int hc_ring(struct hc_slot *s) { assert(s==&slot); rings++; return fail_ring; }
static int hc_complete(struct hc_slot *s,const char *what,bool tur) {
 assert(s==&slot && what && !tur); completions++;
 memset(ucd,0,sizeof(ucd)); ((uint8_t *)ucd)[UCD_RESPONSE+15]=(uint8_t)residual;
 return fail_complete;
}
BACKEND
static void reset(void) {
 memset(&hc_write_window,0,sizeof(hc_write_window));
 hc_usable=hc_in=true; hc_dead=hc_closed=no_slot=false;
 placements=rings=completions=gives=cleans=0;
 fail_place=fail_ring=fail_complete=residual=depth=0;
}
static void enable(void) { assert(ufshc_allow_writes(100,200)==0); }
static void policy(void) {
 struct ufshc_write_window w={100,200,true}; uint8_t u[32], saved[32];
 ufshc_write_command(u,7,100,1);
 assert(ufshc_write_permitted(&w,u,UTRD_CT_UFS|UTRD_DD_OUT));
 w.enabled=false; assert(!ufshc_write_permitted(&w,u,UTRD_CT_UFS|UTRD_DD_OUT)); w.enabled=true;
 assert(!ufshc_write_range(&w,0,99,1)); assert(!ufshc_write_range(&w,0,199,2));
 assert(ufshc_write_range(&w,0,199,1)); assert(ufshc_write_range(&w,0,100,100));
 assert(!ufshc_write_range(&w,0,200,1)); assert(!ufshc_write_range(&w,0,100,0));
 assert(!ufshc_write_range(&w,0,UINT64_MAX,2)); assert(!ufshc_write_range(&w,0,100,UINT64_MAX));
 for(unsigned lun=1;lun<256;lun++)assert(!ufshc_write_range(&w,(uint8_t)lun,100,1));
 memcpy(saved,u,32);
 /* Flip each bit of every byte outside the variable tag/address/count/length. */
 for(unsigned i=0;i<32;i++) {
   if(i==3 || (i>=12&&i<=15) || (i>=18&&i<=21) || i==23 || i==24)continue;
   for(unsigned bit=0;bit<8;bit++) {
     memcpy(u,saved,32);u[i]^=1u<<bit;
     assert(!ufshc_write_permitted(&w,u,UTRD_CT_UFS|UTRD_DD_OUT));
   }
 }
 ufshc_write_command(u,7,100,1);u[15]^=1;
 assert(!ufshc_write_permitted(&w,u,UTRD_CT_UFS|UTRD_DD_OUT));
 ufshc_write_command(u,7,100,1);
 for(unsigned bit=0;bit<32;bit++)assert(!ufshc_write_permitted(&w,u,(UTRD_CT_UFS|UTRD_DD_OUT)^(1u<<bit)));
 ufshc_flush_command(u,7);assert(ufshc_write_permitted(&w,u,UTRD_CT_UFS)); memcpy(saved,u,32);
 for(unsigned i=0;i<32;i++)if(i!=3)for(unsigned bit=0;bit<8;bit++) {
   memcpy(u,saved,32);u[i]^=1u<<bit;assert(!ufshc_write_permitted(&w,u,UTRD_CT_UFS));
 }
 w.first=0xffffffffULL;w.end=0x100000000ULL;
 ufshc_write_command(u,1,0xffffffffu,1);assert(ufshc_write_permitted(&w,u,UTRD_CT_UFS|UTRD_DD_OUT));
 w.end++;assert(!ufshc_write_permitted(&w,u,UTRD_CT_UFS|UTRD_DD_OUT));
}
int main(void) {
 uint8_t src[12288];memset(src,0xa5,sizeof(src));policy();reset();
 assert(ufshc_write(0,100ULL*BLOCK,src,BLOCK)==EROFS);assert(ufshc_flush()==EIO);assert(!rings);
 assert(ufshc_allow_writes(200,100)==EINVAL);assert(ufshc_allow_writes(0,100)==EINVAL);
 assert(ufshc_allow_writes(100,0x100000001ULL)==EINVAL);enable();
 assert(ufshc_allow_writes(1,1000)==EINVAL);
 assert(ufshc_write(0,99ULL*BLOCK,src,BLOCK)==EROFS);
 assert(ufshc_write(0,199ULL*BLOCK,src,2*BLOCK)==EROFS);
 assert(ufshc_write(1,100ULL*BLOCK,src,BLOCK)==EROFS);
 assert(ufshc_write(0,100ULL*BLOCK+1,src,BLOCK)==EROFS);
 assert(ufshc_write(0,100ULL*BLOCK,src,1)==EROFS);
 assert(ufshc_write(0,100ULL*BLOCK,src,UINT64_MAX)==EROFS);assert(!rings);
 assert(ufshc_write(0,100ULL*BLOCK,src,sizeof(src))==0);
 assert(rings==2 && completions==2 && gives==2 && cleans==2);
 assert(commands[0].upiu[24]==2 && commands[1].upiu[21]==102 && commands[1].upiu[24]==1);
 assert(ufshc_flush()==0);assert(rings==3 && gives==3 && commands[2].upiu[16]==0x35);
 reset();enable();fail_place=EPERM;assert(ufshc_write(0,100ULL*BLOCK,src,BLOCK)==EPERM);
 assert(!rings && !hc_dead && gives==1);
 reset();enable();fail_ring=ENXIO;assert(ufshc_write(0,100ULL*BLOCK,src,BLOCK)==ENXIO);
 assert(!completions && !hc_dead && gives==1);
 reset();enable();fail_complete=EIO;assert(ufshc_write(0,100ULL*BLOCK,src,sizeof(src))==EIO);
 assert(hc_dead && rings==1 && gives==1);
 assert(ufshc_write(0,100ULL*BLOCK,src,BLOCK)==EIO && rings==1);
 reset();enable();fail_complete=ETIMEDOUT;assert(ufshc_write(0,100ULL*BLOCK,src,BLOCK)==ETIMEDOUT);
 assert(hc_dead && !gives && rings==1); /* DMA slot quarantined */
 reset();enable();residual=1;assert(ufshc_write(0,100ULL*BLOCK,src,BLOCK)==EIO);assert(hc_dead);
 reset();enable();fail_complete=EIO;assert(ufshc_flush()==EIO);assert(hc_dead && rings==1);
 reset();enable();no_slot=true;assert(ufshc_flush()==EIO && !rings);
 puts("native UFS: command/range guards, chunking, flush, DMA preparation, failures and timeout quarantine passed");
}
'''.replace('BACKEND', backend)
with tempfile.TemporaryDirectory() as temp:
    path = Path(temp)
    (path / 'test.c').write_text(harness)
    subprocess.run(['clang', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-I' + str(root / 'drivers/storage'),
                    str(path / 'test.c'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)
