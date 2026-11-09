#!/usr/bin/env python3
"""Actual storage dispatch/flush functions and loader ABI, tested without hardware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
source = (root / 'drivers/storage/hakios.ufs.c').read_text()
loader = (root / 'xnuloader/loader.c').read_text()

def section(text, first, last):
    start = text.index(first)
    return text[start:text.index(last, start)]

functions = '\n'.join([
    section(source, 'static int\nufs_read(uint32_t', '/*\n * Whole blocks'),
    section(source, 'static int\nufs_write(uint32_t', '/* The writing over:'),
    section(source, 'static uint32_t\nufs_data_op(', '/*\n * Bytes [offset, offset + len) of the data volume'),
])
structs = ''
for name, text in [('kernel', source), ('loader', loader)]:
    declarations = section(text, 'struct loader_block_part {', '_Static_assert')
    structs += declarations.replace('loader_block', name + '_block')

harness = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define BLOCK_MAX_PARTS 160
STRUCTS
_Static_assert(sizeof(struct kernel_block)==sizeof(struct loader_block),"shared size");
_Static_assert(offsetof(struct kernel_block,native)==offsetof(struct loader_block,native),"native offset");
_Static_assert(offsetof(struct kernel_block,loader_native)==offsetof(struct loader_block,loader_native),"padding preserved");
_Static_assert(offsetof(struct kernel_block,kernel_native)==offsetof(struct loader_block,kernel_native),"kernel offset");
_Static_assert(sizeof(((struct kernel_block *)0)->native)<=128,"ABI extension fits padding");
_Static_assert(offsetof(struct kernel_block,native.gpt_pa)==offsetof(struct loader_block,native.gpt_pa),"snapshot ABI");
_Static_assert(offsetof(struct kernel_block,native.gpt_mask)==offsetof(struct loader_block,native.gpt_mask),"snapshot mask ABI");
#define UFS_WRITE_FIRST 2042368ULL
#define BLOCK_OP_READ 0u
#define BLOCK_OP_WRITE 1u
#define BLOCK_OP_WRITE_FAST 2u
#define BLOCK_OP_FLUSH 3u
#define BLOCK_CAN_FAST 1u
#define MIN(a,b) ((a)<(b)?(a):(b))
#define Z_WAITOK 0
static struct kernel_block page, *ufs_page=&page;
static unsigned ufs_parts=2, ufs_buffer_size=8192;
static uint8_t ufs_lun[2];
static uint64_t ufs_lun_offset[2];
static bool ufs_ready, ufs_native_required, ufs_native_writes, ufs_native_on;
static bool ufs_write_out, ufs_write_was_native, ufs_write_busy;
static int ufs_write_part=1, ufs_write_mtx, ufs_lock, ufs_data_mtx;
static uint64_t ufs_native_fallbacks, ufs_writes, ufs_write_bytes, ufs_write_errors, ufs_write_refused, ufs_native_flushes;
static unsigned writes, reads, flushes, mailbox, switches, fail_write_at;
static int read_error, write_error, flush_error;
static bool corrupt, no_memory, list_in, expect_data_lock;
static uint8_t storage[32768];
static void lck_mtx_lock(int *l) { assert(!*l);*l=1; }
static void lck_mtx_unlock(int *l) { assert(*l);*l=0; }
static void *kalloc_data(size_t n,int flags) { (void)flags;return no_memory?NULL:malloc(n); }
static void kfree_data(void *p,size_t n) { (void)n;free(p); }
static uint64_t ufs_part_bytes(uint32_t p) { assert(p<ufs_parts);return page.part[p].blocks*4096; }
static int ufs_read_native(uint32_t p,uint64_t at,uint8_t *dst,uint64_t n) {
 assert(p==1);reads++;if(read_error)return read_error;
 assert(at>=UFS_WRITE_FIRST*4096 && at-UFS_WRITE_FIRST*4096+n<=sizeof(storage));
 memcpy(dst,storage+(at-UFS_WRITE_FIRST*4096),n);if(corrupt && n)dst[0]^=1;return 0;
}
static int ufs_read_mailbox(uint32_t p,uint64_t at,uint8_t *dst,uint64_t n) {
 (void)p;(void)at;(void)dst;(void)n;mailbox++;return 0;
}
static int ufs_native_out(const char *why) { assert(why);switches++;list_in=false;return 0; }
static bool ufshc_list_in(void) { return list_in; }
static int ufs_request(uint32_t op,uint32_t p,uint64_t at,uint64_t n,uint8_t *dst,const uint8_t *src) {
 (void)op;(void)p;(void)at;(void)n;(void)dst;(void)src;mailbox++;return 0;
}
static int ufshc_write(uint8_t lun,uint64_t at,const uint8_t *src,uint64_t n) {
 assert(lun==0);writes++;if(write_error && writes==fail_write_at)return write_error;
 uint64_t relative=at-ufs_lun_offset[1]-UFS_WRITE_FIRST*4096;
 assert(relative+n<=sizeof(storage));memcpy(storage+relative,src,n);return 0;
}
static int ufshc_flush(void) {
 assert(ufs_write_mtx);if(expect_data_lock)assert(ufs_data_mtx);
 flushes++;return flush_error;
}
FUNCTIONS
static void reset(void) {
 memset(&page,0,sizeof(page));page.part[1].block_size=4096;page.part[1].blocks=UFS_WRITE_FIRST+8;
 page.write_caps=BLOCK_CAN_FAST;
 memset(storage,0,sizeof(storage));ufs_lun[0]=0xff;ufs_lun[1]=0;ufs_lun_offset[1]=100ULL*4096;
 ufs_ready=ufs_native_required=ufs_native_writes=ufs_native_on=list_in=true;
 ufs_write_out=ufs_write_was_native=ufs_write_busy=corrupt=no_memory=expect_data_lock=false;
 writes=reads=flushes=mailbox=switches=0;fail_write_at=1;
 read_error=write_error=flush_error=0;
 ufs_writes=ufs_write_bytes=ufs_write_errors=ufs_write_refused=ufs_native_fallbacks=ufs_native_flushes=0;
}
int main(void) {
 uint8_t src[12288],dst[4096];memset(src,0x5a,sizeof(src));reset();
 assert(ufs_write(BLOCK_OP_WRITE_FAST,1,UFS_WRITE_FIRST*4096,src,sizeof(src))==0);
 assert(writes==2 && !flushes && !mailbox && !switches && list_in && ufs_native_on);
 assert(ufs_writes==2 && ufs_write_bytes==sizeof(src));
 expect_data_lock=true;
 assert(ufs_data_flush()==0 && flushes==1 && ufs_native_flushes==1 && !mailbox);
 assert(!ufs_data_mtx && !ufs_write_mtx);expect_data_lock=false;
 assert(ufs_read(1,UFS_WRITE_FIRST*4096,dst,sizeof(dst))==0 && !memcmp(dst,src,sizeof(dst)));
 read_error=EIO;assert(ufs_read(1,UFS_WRITE_FIRST*4096,dst,sizeof(dst))==EIO && !mailbox && !switches);
 assert(ufs_read(2,0,dst,1)==EINVAL);assert(ufs_read(1,ufs_part_bytes(1),dst,1)==EINVAL);
 reset();assert(ufs_write(BLOCK_OP_WRITE,1,UFS_WRITE_FIRST*4096,src,sizeof(src))==0);
 assert(writes==2 && flushes==2 && reads==2 && !mailbox);
 reset();corrupt=true;assert(ufs_write(BLOCK_OP_WRITE,1,UFS_WRITE_FIRST*4096,src,sizeof(src))==EIO);
 assert(writes==1 && flushes==1 && reads==1 && ufs_write_errors==1 && !mailbox);
 reset();write_error=ETIMEDOUT;assert(ufs_write(BLOCK_OP_WRITE_FAST,1,UFS_WRITE_FIRST*4096,src,sizeof(src))==ETIMEDOUT);
 assert(writes==1 && !mailbox && !switches && ufs_write_errors==1);
 reset();flush_error=EIO;assert(ufs_data_flush()==EIO && ufs_write_errors==1 && !mailbox);
 reset();flush_error=EIO;assert(ufs_write(BLOCK_OP_WRITE,1,UFS_WRITE_FIRST*4096,src,sizeof(src))==EIO);
 assert(writes==1 && flushes==1 && !reads && !mailbox);
 reset();no_memory=true;assert(ufs_write(BLOCK_OP_WRITE,1,UFS_WRITE_FIRST*4096,src,sizeof(src))==ENOMEM && !writes);
 reset();ufs_native_writes=false;assert(ufs_write(BLOCK_OP_WRITE_FAST,1,UFS_WRITE_FIRST*4096,src,4096)==EIO && !writes && !mailbox);
 reset();assert(ufs_write(BLOCK_OP_WRITE_FAST,1,(UFS_WRITE_FIRST-1)*4096,src,4096)==EROFS && !writes);
 assert(ufs_write(BLOCK_OP_WRITE_FAST,0,UFS_WRITE_FIRST*4096,src,4096)==EROFS && !writes);
 assert(ufs_write(BLOCK_OP_WRITE_FAST,1,(UFS_WRITE_FIRST+8)*4096,src,4096)==EROFS && !writes);
 reset();ufs_native_required=ufs_native_writes=false;
 assert(ufs_write(BLOCK_OP_WRITE_FAST,1,UFS_WRITE_FIRST*4096,src,4096)==0);
 assert(mailbox==1 && switches==1 && !writes && !ufs_native_on);
 puts("native UFS storage: shared ABI, native dispatch, raw read-back, ordered flush routing, no fallback, failures and legacy routing passed");
}
'''.replace('STRUCTS', structs).replace('FUNCTIONS', functions)
with tempfile.TemporaryDirectory() as temp:
    path = Path(temp)
    (path / 'test.c').write_text(harness)
    # Darwin's uint64_t uses unsigned long long; Linux uses unsigned long.
    subprocess.run(['clang', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-Wno-format',
                    '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                    str(path / 'test.c'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)
