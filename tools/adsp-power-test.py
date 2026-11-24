#!/usr/bin/env python3
"""Run the production ADSP rail sender with completion and failure injection."""
from pathlib import Path
import subprocess, tempfile
s=(Path(__file__).resolve().parents[1]/'drivers/audio/hakios.adsp.c').read_text()
start=s.index('static bool rails_asked;')
end=s.index('/* ---- step 1:',start)
production=s[start:end]
prefix=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#define RPMH_TCS 0
#define RSC_TCS(n) (0x100+(n)*0x100)
#define RSC_IRQ_CLEAR 8
#define RSC_IRQ_STATUS 4
#define TCS_STATUS 0x18
#define TCS_CONTROL 0x14
#define TCS_COMMAND(n) (0x30+20*(n))
#define TCS_WAIT_FOR_DONE 0x10
#define TCS_ENABLED 0x1c
#define TCS_TRIGGER (1u<<24)
#define TCS_ACTIVE_MODE (1u<<16)
#define MSGID_WRITE_WAITED 0x10108
#define lck_mtx_lock(x) ((void)(x))
#define lck_mtx_unlock(x) ((void)(x))
#define printf(...) ((void)0)
static unsigned step, hakios_rsc_lock;
static bool leaving;
static char last_write[256];
static struct { unsigned base; } rsc;
static unsigned regs[512], reply=1, cmd[2]={0x10101,0x10101};
static uint64_t clock_us;
static int missing;
static int rail_position(const char *n,uint32_t l,uint32_t *a) {
 *a=missing?0:(!strcmp(n,"lcx.lvl")?0x30000:0x30004); return l?3:0;
}
static void need(void *p) {(void)p;}
static void hold(void) {}
static void settle(void) {}
static uint64_t now_us(void) {return clock_us;}
static void sleep_us(unsigned t) {clock_us+=t;}
static uint32_t rd(unsigned a) {return regs[a/4];}
static void wr(unsigned a,unsigned v) {
 regs[a/4]=v;
 if(a==RSC_IRQ_CLEAR) regs[RSC_IRQ_STATUS/4]=0;
 if(a==RSC_TCS(0)+TCS_CONTROL && (v&TCS_TRIGGER)) {
  regs[RSC_IRQ_STATUS/4]=reply;
  for(unsigned i=0;i<2;i++) regs[(RSC_TCS(0)+TCS_COMMAND(i)+12)/4]=cmd[i];
 }
}
'''
test=r'''
int main(void) {
 regs[(RSC_TCS(0)+TCS_STATUS)/4]=1;
 assert(rails(384)&&rails_asked&&!rail_trace.uncertain);
 reply=0;
 assert(!rails(0)&&rails_asked&&rail_trace.uncertain&&rail_trace.error==ETIMEDOUT);
 assert(rail_trace.acknowledged==384&&rail_trace.release_failures==1);
 reply=1; assert(rails(0)&&!rails_asked&&!rail_trace.uncertain);
 assert(rail_trace.ack_valid&&rail_trace.acknowledged==0);
 // A submitted acquisition may reach hardware even if its response is lost.
 reply=0; assert(!rails(384)&&rails_asked&&rail_trace.uncertain);
 reply=1; assert(rails(0)&&!rails_asked);
 assert(rails(384));
 cmd[1]=0x101; assert(!rails(0)&&rails_asked&&rail_trace.error==EIO);
 cmd[1]=~0u; assert(!rails(0)&&rails_asked&&rail_trace.error==EIO);
 cmd[1]=0x10101;
 regs[(RSC_TCS(0)+TCS_STATUS)/4]=0;
 unsigned submitted=rail_trace.submitted;
 assert(!rails(0)&&rails_asked&&rail_trace.error==EBUSY);
 assert(rail_trace.submitted==submitted);
 regs[(RSC_TCS(0)+TCS_STATUS)/4]=1;
 missing=1; assert(!rails(0)&&rails_asked&&rail_trace.error==ENXIO);
 missing=0; assert(rails(0)&&!rails_asked);
 missing=1; assert(!rails(384)&&!rails_asked);
 assert(rail_trace.failures==7&&rail_trace.release_failures==5);
 puts("ADSP production rail sender: release timeout, acquisition timeout, partial/dead completion, busy/missing resource and acknowledged recovery passed");
}
'''
with tempfile.TemporaryDirectory(prefix='adsp-power-') as d:
 p=Path(d);(p/'test.c').write_text(prefix+production+test)
 subprocess.run(['clang','-std=gnu11','-O1','-g','-Wall','-Wextra','-Werror','-Wno-unused-variable','-fsanitize=address,undefined','-fno-sanitize-recover=all',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
