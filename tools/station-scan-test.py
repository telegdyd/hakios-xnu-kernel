#!/usr/bin/env python3
"""Compile the production scan-stage wait against an always-busy iOS client."""
from pathlib import Path
import subprocess
import tempfile
source=(Path(__file__).resolve().parents[1]/'drivers/wifi/hakios.wifi.pci.c').read_text()
a=source.index('static int\nop_scan(');b=source.index('/* The network (the words',a)
code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#define __unused __attribute__((unused))
#define Z_WAITOK 0
#define Z_ZERO 0
#define NSEC_PER_MSEC 1
#define hakios_sta_busy() 1
#define kalloc_type(t,f) (mode==3?NULL:calloc(1,sizeof(t)))
#define kfree_type(t,p) free(p)
struct hakios_sta_scan {bool passive;};
struct hakios_sta_bss {int dummy;};
static int mode,ticks;
static uint64_t generation;
static bool hakios_sta_present(void){return mode!=1;}
static uint64_t hakios_sta_scan_generation(void){return generation;}
static int hakios_sta_ask_scan(const struct hakios_sta_scan *s){assert(s->passive);return mode==2?EBUSY:0;}
static void delay_for_interval(int a,int b){(void)a;(void)b;ticks++;if(ticks==2&&mode!=4)generation++;}
static bool hakios_sta_heard(unsigned i,struct hakios_sta_bss *b){(void)b;return i<3;}
'''+source[a:b]+r'''
int main(void){unsigned bss=0;mode=0;ticks=0;generation=10;
 assert(op_scan(NULL,&bss)==0&&bss==3&&ticks==2); /* other queued work never goes idle */
 mode=1;assert(op_scan(NULL,&bss)==ENXIO);
 mode=2;assert(op_scan(NULL,&bss)==EBUSY);
 mode=3;ticks=0;assert(op_scan(NULL,&bss)==ENOMEM);
 mode=4;ticks=0;assert(op_scan(NULL,&bss)==ETIMEDOUT&&ticks==600);
 puts("station scan: completion despite queued scans, absent radio, request/allocation failures and bounded timeout pass");
}
'''
with tempfile.TemporaryDirectory(prefix='hakios-scan-') as tmp:
 p=Path(tmp);(p/'test.c').write_text(code)
 subprocess.run(['clang','-std=gnu11','-O1','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
