#!/usr/bin/env python3
"""Independent integer/affine P-256 oracle; no production implementation used.
Emit deterministic randomized SAE and field cases. Python hashlib/hmac provide
the hash oracle; curve operations use Python arbitrary precision integers.
"""
import hashlib
import hmac
import random
import sys

P = 2**256 - 2**224 + 2**192 + 2**96 - 1
N = int('ffffffff00000000ffffffffffffffffbce6faada7179e84f3b9cac2fc632551', 16)
B = int('5ac635d8aa3a93e7b3ebbd55769886bc651d06b0cc53b0f63bce3c3e27d2604b', 16)
def enc(x, n=32): return x.to_bytes(n, 'big')
def mac(key, data): return hmac.digest(key, data, 'sha256')
def kdf(key, label, context, length):
    return b''.join(mac(key, i.to_bytes(2, 'little')+label+context+(length*8).to_bytes(2, 'little'))
                    for i in range(1, (length+31)//32+1))[:length]
def expand(key, label, length):
    t, result = b'', b''
    for i in range(1, (length+31)//32+1):
        t = mac(key, t+label+bytes([i])); result += t
    return result[:length]
def add(p, q):
    if p is None: return q
    if q is None: return p
    x, y = p; u, v = q
    if x == u and (y+v) % P == 0: return None
    slope = ((3*x*x-3)*pow(2*y, -1, P) if p == q else (v-y)*pow(u-x, -1, P)) % P
    z = (slope*slope-x-u) % P
    return z, (slope*(x-z)-y) % P
def mul(p, k):
    out = None
    while k:
        if k & 1: out = add(out, p)
        p = add(p, p); k >>= 1
    return out
def curve(x): return (x*x*x-3*x+B) % P
def sswu(u):
    z, a = P-10, P-3
    m = (z*z*pow(u,4,P)+z*u*u) % P
    x1 = (B*pow(z*a,-1,P) if m == 0 else -B*pow(a,-1,P)*(1+pow(m,-1,P))) % P
    x2 = z*u*u*x1 % P
    g = curve(x1)
    x = x1 if pow(g, (P-1)//2, P) in (0,1) else x2
    y = pow(curve(x), (P+1)//4, P)
    if (y ^ u) & 1: y = P-y
    return x, y
def pwe(own, peer, ssid, password, h2e):
    addresses = max(own,peer)+min(own,peer)
    if h2e:
        seed = mac(ssid,password)
        p1, p2 = (sswu(int.from_bytes(expand(seed,label,48),'big') % P)
                  for label in (b'SAE Hash to Element u1 P1', b'SAE Hash to Element u2 P2'))
        val = int.from_bytes(mac(bytes(32),addresses),'big') % (N-1)+1
        return mul(add(p1,p2),val)
    for counter in range(1,256):
        seed = mac(addresses,password+bytes([counter]))
        x = int.from_bytes(kdf(seed,b'SAE Hunting and Pecking',enc(P),32),'big')
        if x >= P: continue
        y = pow(curve(x),(P+1)//4,P)
        if y*y % P != curve(x): continue
        if (y ^ seed[-1]) & 1: y = P-y
        return x, y
def point(p): return enc(p[0])+enc(p[1])
def array(b): return '{'+','.join(str(v) for v in b)+'}'

rng = random.Random(int(sys.argv[1]) if len(sys.argv)>1 else 1)
print('static const struct math_case { unsigned mod; uint8_t a[32],b[32],sum[32],diff[32],product[32]; } math_cases[] = {')
for m in (P,N):
    for _ in range(80):
        a, b = rng.randrange(m), rng.randrange(m)
        print('{'+str(int(m==N))+','+','.join(array(enc(v)) for v in (a,b,(a+b)%m,(a-b)%m,a*b%m))+'},')
print('};')
print('static const struct sae_case { unsigned h2e; uint8_t own[6],peer[6],password[16],random[32],mask[32],pwe[64],commit[98],peer_commit[98],kck[32],pmk[32],pmkid[16],confirm[34]; } cases[] = {')
for mode in (0,1):
    for _ in range(3):
        own, peer, password = rng.randbytes(6), rng.randbytes(6), rng.randbytes(16)
        r, mask, pr, pmask = (rng.randrange(1,N) for _ in range(4))
        pw = pwe(own,peer,b'testssid',password,mode)
        def commit(r,m):
            e = mul(pw,m)
            return b'\x13\x00'+enc((r+m)%N)+point((e[0],P-e[1]))
        co, cp = commit(r,mask), commit(pr,pmask)
        k = mul(pw,r*pr%N)[0]
        context = enc((r+mask+pr+pmask)%N)
        keys = kdf(mac(bytes(32),enc(k)),b'SAE KCK and PMK',context,64)
        # Peer uses send-confirm 0, as hostap does on its first confirm.
        conf = b'\x00\x00'+mac(keys[:32],b'\x00\x00'+cp[2:]+co[2:])
        print('{'+str(mode)+','+','.join(array(v) for v in (own,peer,password,enc(r),enc(mask),point(pw),co,cp,keys[:32],keys[32:],context[:16],conf))+'},')
print('};')
