"""Deliver firmware HELLO synchronously at the first production RX queue call."""
from pathlib import Path
import subprocess,tempfile,sys
r=Path(__file__).resolve().parents[1]
source=Path(sys.argv[1]) if len(sys.argv)>1 else r/'wifi/lx/src/lx_mhi.c'
s=source.read_text();s=s[s.index('static int\nipcr_probe('):s.index('\nstatic void\nipcr_remove(')]
prefix=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
typedef uint32_t u32;
#define GFP_KERNEL 0
#define DMA_FROM_DEVICE 0
#define MHI_EOT 0
#define MHI_LOG(...) ((void)0)
struct mhi_controller {unsigned buffer_len;};
struct mhi_device {void *ul_chan,*dl_chan;struct mhi_controller *mhi_cntrl;};
struct mhi_device_id {int unused;};
struct link {void *send,*ctx;const char *name;};
static struct {struct link link;struct mhi_device *dev;bool up;void **rx;unsigned rx_count,rx_len;} ipcr;
static int allocations,alloc_calls,alloc_fail=-1,queue_calls,queue_fail=-1,link_fail,linked,hellos,early,unprepares;
static void *kzalloc(size_t n,int flags){if(alloc_calls++==alloc_fail)return NULL;allocations++;return calloc(1,n);}
static void ipcr_free_rx(void){for(unsigned i=0;i<ipcr.rx_count;i++){if(ipcr.rx[i]){free(ipcr.rx[i]);allocations--;}}if(ipcr.rx){free(ipcr.rx);allocations--;}ipcr.rx=NULL;ipcr.rx_count=0;}
static int ipcr_send(void){return 0;}
static int mhi_prepare_for_transfer(struct mhi_device *d){return 0;}
static int mhi_get_free_desc_count(struct mhi_device *d,int dir){return 3;}
static void mhi_unprepare_from_transfer(struct mhi_device *d){unprepares++;}
static int lx_os_qrtr_link(struct link *l){if(l&&link_fail)return EIO;linked=!!l;return 0;}
static int mhi_queue_buf(struct mhi_device *d,int dir,void *b,unsigned n,int flags){
 int call=queue_calls++;if(call==queue_fail)return -EIO;
 if(!call){if(linked&&ipcr.up&&ipcr.dev==d)hellos++;else early++;}
 return 0;
}
'''
tests=r'''
static void reset(void){assert(allocations==0);alloc_calls=queue_calls=link_fail=linked=hellos=early=unprepares=0;alloc_fail=queue_fail=-1;}
int main(void){
 struct mhi_controller c={8192};struct mhi_device d={(void*)1,(void*)1,&c};
 assert(ipcr_probe(&d,NULL)==0);
 if(early){puts("REPRODUCED: first firmware HELLO dropped before router publication");ipcr_free_rx();return 9;}
 assert(hellos==1&&linked&&ipcr.up);ipcr.up=false;ipcr.dev=NULL;linked=0;ipcr_free_rx();
 for(int i=0;i<4;i++){reset();alloc_fail=i;assert(ipcr_probe(&d,NULL)==-ENOMEM);assert(!ipcr.dev&&!ipcr.up&&!linked&&allocations==0&&unprepares==1);}
 reset();link_fail=1;assert(ipcr_probe(&d,NULL)==-EIO);assert(!ipcr.dev&&!linked&&allocations==0);
 for(int i=0;i<3;i++){reset();queue_fail=i;assert(ipcr_probe(&d,NULL)==-EIO);assert(!ipcr.dev&&!ipcr.up&&!linked&&allocations==0&&unprepares==1);}
 puts("ipcr-publication: earliest HELLO delivered; allocation/router/partial-queue failures unwind cleanly");
}
'''
with tempfile.TemporaryDirectory(prefix='ipcr-publication-') as tmp:
 p=Path(tmp);(p/'test.c').write_text(prefix+s+tests)
 subprocess.run(['clang','-std=gnu11','-O1','-g','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
