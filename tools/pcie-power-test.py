#!/usr/bin/env python3
"""Exercise production TCS reuse checks against ABL and in-flight register states."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
source = (root/'drivers/wifi/hakios.pcie.c').read_text()
start = source.index('static int\ntcs_idle(')
end = source.index('static int\nclaim_tcs(', start)
prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#define TCS_BASE 0xd00u
#define TCS_STRIDE 0x2a0u
#define CHECK(x) do { int e=(x); if(e) return e; } while(0)
static unsigned ncpt=16;
static uint8_t memory[0x2000], *rsc=memory;
static int rd(void *p,unsigned *v) { *v=*(unsigned *)p; return 0; }
static void reg(unsigned off,unsigned v) { *(unsigned *)(rsc+TCS_BASE+off)=v; }
'''
tests = r'''
int main(void) {
 reg(0x18,1); reg(0x14,0x1010000); reg(0x1c,3); reg(0x10,0);
 reg(0x3c,0x10101); reg(0x50,0x101);
 assert(tcs_idle(0)==0); /* ABL's completed ACTIVE request, wait=0 */
 reg(0x10,3); assert(tcs_idle(0)==EBUSY); /* response still outstanding */
 reg(0x50,0x10101); assert(tcs_idle(0)==0);
 reg(0x18,0); assert(tcs_idle(0)==EBUSY);
 reg(0x18,~0u); assert(tcs_idle(0)==EBUSY);
 reg(0x18,1); reg(0x14,1u<<24); assert(tcs_idle(0)==EBUSY); /* armed sleep/wake */
 reg(0x14,0x1010000); reg(0x50,~0u); assert(tcs_idle(0)==EBUSY);
 reg(0x50,0); reg(0x10,0); assert(tcs_idle(0)==EBUSY);
 reg(0x50,0x101); reg(0x1c,1u<<16); assert(tcs_idle(0)==EBUSY);
 reg(0x1c,0); reg(0x14,0); assert(tcs_idle(0)==0);
 puts("pcie-power: 10 completed/armed/busy/dead-link TCS reuse cases passed");
}
'''
with tempfile.TemporaryDirectory(prefix='pcie-power-') as tmp:
 p=Path(tmp)
 (p/'test.c').write_text(prefix+source[start:end]+tests)
 subprocess.run(['clang','-std=gnu11','-O1','-g','-Wall','-Wextra','-Werror',
 '-fsanitize=address,undefined','-fno-sanitize-recover=all',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
