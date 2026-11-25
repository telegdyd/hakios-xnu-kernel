; img_store: one typed 2D image store through IBO 0, the narrowest test of an image path (no preamble, no AIR
; front end): the probe's --case img_store. Written non-bindless (.imm); on gen 7 tools/ir3-kernel makes the store
; bindless set 0 (.imm.base0), since the A730 runs non-bindless IBOs unreliably (GPU-PLAN.md, 10 November).
;
; SPDX-License-Identifier: MIT. Ours; the syntax is fill_id.asm's.
;
; A 64-pixel-wide R32Uint image, 8x8 groups: x = wgid.x * 8 + lid.x, y = wgid.y * 8 + lid.y, i = y * 64 + x, and the
; pixel gets i * K + c (fill_id's word: which invocation wrote it). c is c0.x (the probe sets it). No lock/unlock
; before end (ir3 adds them on gen 7, the assembler is gen 6 too); fill_id.asm runs on the A730 without them.

@localsize 8, 8, 1
@const(c0.x) 0x00001234, 0, 0, 0
@wgid(r48.x)
@invocationid(r0.x)

shl.b r1.x, r48.x, 3
shl.b r1.y, r48.y, 3
mov.u32u32 r2.x, 0x9e3779b1
(rpt2)nop
add.u r1.x, r1.x, r0.x           ; x
add.u r1.y, r1.y, r0.y           ; y
(rpt2)nop
shl.b r3.y, r1.y, 6
(rpt2)nop
add.u r3.y, r3.y, r1.x           ; i
(rpt2)nop
mull.u r3.x, r3.y, r2.x
(rpt2)nop
madsh.m16 r3.x, r3.y, r2.x, r3.x
(rpt2)nop
madsh.m16 r3.x, r2.x, r3.y, r3.x
(rpt2)nop
add.u r3.x, r3.x, c0.x
(rpt5)nop
stib.b.typed.2d.u32.1.imm r3.x, r1.x, 0
end
