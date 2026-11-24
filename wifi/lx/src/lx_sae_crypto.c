/*
 * SHA-256 hash implementation and interface functions
 * Copyright (c) 2003-2011, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

/* Adapted for lx: fixed-width types, local endian helpers, names private. */
#ifdef _KERNEL
#include <sys/param.h>
#include <sys/systm.h>
#else
#include <string.h>
#endif
#include "lx_sae_crypto.h"
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
#define os_memcpy memcpy
#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif
#define TEST_FAIL() 0
#define SHA256_BLOCK_SIZE 64
#define WPA_GET_BE32(b) ((uint32_t)(b)[0]<<24|(uint32_t)(b)[1]<<16|(uint32_t)(b)[2]<<8|(b)[3])
#define WPA_PUT_BE32(b,v) do { uint32_t value=(v); (b)[0]=value>>24;(b)[1]=value>>16;(b)[2]=value>>8;(b)[3]=value; } while(0)
#define WPA_PUT_BE64(b,v) do { uint64_t value64=(v);for(unsigned ii=0;ii<8;ii++) (b)[7-ii]=(uint8_t)(value64>>(8*ii)); } while(0)
struct sha256_state { u64 length;u32 state[8],curlen;u8 buf[64]; };
static void sha256_init(struct sha256_state *);
static int sha256_process(struct sha256_state *,const unsigned char *,unsigned long);
static int sha256_done(struct sha256_state *,unsigned char *);
/**
 * sha256_vector - SHA256 hash for data vector
 * @num_elem: Number of elements in the data vector
 * @addr: Pointers to the data areas
 * @len: Lengths of the data blocks
 * @mac: Buffer for the hash
 * Returns: 0 on success, -1 of failure
 */
static int sha256_vector(size_t num_elem, const u8 *const addr[], const size_t *len,
		  u8 *mac)
{
	struct sha256_state ctx;
	size_t i;

	if (TEST_FAIL())
		return -1;

	sha256_init(&ctx);
	for (i = 0; i < num_elem; i++)
		if (sha256_process(&ctx, addr[i], len[i]))
			return -1;
	if (sha256_done(&ctx, mac))
		return -1;
	lx_crypto_wipe(&ctx, sizeof(ctx));
	return 0;
}


/* ===== start - public domain SHA256 implementation ===== */

/* This is based on SHA256 implementation in LibTomCrypt that was released into
 * public domain by Tom St Denis. */

/* the K array */
static const unsigned long K[64] = {
	0x428a2f98UL, 0x71374491UL, 0xb5c0fbcfUL, 0xe9b5dba5UL, 0x3956c25bUL,
	0x59f111f1UL, 0x923f82a4UL, 0xab1c5ed5UL, 0xd807aa98UL, 0x12835b01UL,
	0x243185beUL, 0x550c7dc3UL, 0x72be5d74UL, 0x80deb1feUL, 0x9bdc06a7UL,
	0xc19bf174UL, 0xe49b69c1UL, 0xefbe4786UL, 0x0fc19dc6UL, 0x240ca1ccUL,
	0x2de92c6fUL, 0x4a7484aaUL, 0x5cb0a9dcUL, 0x76f988daUL, 0x983e5152UL,
	0xa831c66dUL, 0xb00327c8UL, 0xbf597fc7UL, 0xc6e00bf3UL, 0xd5a79147UL,
	0x06ca6351UL, 0x14292967UL, 0x27b70a85UL, 0x2e1b2138UL, 0x4d2c6dfcUL,
	0x53380d13UL, 0x650a7354UL, 0x766a0abbUL, 0x81c2c92eUL, 0x92722c85UL,
	0xa2bfe8a1UL, 0xa81a664bUL, 0xc24b8b70UL, 0xc76c51a3UL, 0xd192e819UL,
	0xd6990624UL, 0xf40e3585UL, 0x106aa070UL, 0x19a4c116UL, 0x1e376c08UL,
	0x2748774cUL, 0x34b0bcb5UL, 0x391c0cb3UL, 0x4ed8aa4aUL, 0x5b9cca4fUL,
	0x682e6ff3UL, 0x748f82eeUL, 0x78a5636fUL, 0x84c87814UL, 0x8cc70208UL,
	0x90befffaUL, 0xa4506cebUL, 0xbef9a3f7UL, 0xc67178f2UL
};


/* Various logical functions */
#define RORc(x, y) \
( ((((unsigned long) (x) & 0xFFFFFFFFUL) >> (unsigned long) ((y) & 31)) | \
   ((unsigned long) (x) << (unsigned long) (32 - ((y) & 31)))) & 0xFFFFFFFFUL)
#define Ch(x,y,z)       (z ^ (x & (y ^ z)))
#define Maj(x,y,z)      (((x | y) & z) | (x & y))
#define S(x, n)         RORc((x), (n))
#define R(x, n)         (((x)&0xFFFFFFFFUL)>>(n))
#define Sigma0(x)       (S(x, 2) ^ S(x, 13) ^ S(x, 22))
#define Sigma1(x)       (S(x, 6) ^ S(x, 11) ^ S(x, 25))
#define Gamma0(x)       (S(x, 7) ^ S(x, 18) ^ R(x, 3))
#define Gamma1(x)       (S(x, 17) ^ S(x, 19) ^ R(x, 10))

/* compress 512-bits */
static int sha256_compress(struct sha256_state *md, unsigned char *buf)
{
	u32 S[8], W[64], t0, t1;
	u32 t;
	int i;

	/* copy state into S */
	for (i = 0; i < 8; i++) {
		S[i] = md->state[i];
	}

	/* copy the state into 512-bits into W[0..15] */
	for (i = 0; i < 16; i++)
		W[i] = WPA_GET_BE32(buf + (4 * i));

	/* fill W[16..63] */
	for (i = 16; i < 64; i++) {
		W[i] = Gamma1(W[i - 2]) + W[i - 7] + Gamma0(W[i - 15]) +
			W[i - 16];
	}

	/* Compress */
#define RND(a,b,c,d,e,f,g,h,i)                          \
	t0 = h + Sigma1(e) + Ch(e, f, g) + K[i] + W[i];	\
	t1 = Sigma0(a) + Maj(a, b, c);			\
	d += t0;					\
	h  = t0 + t1;

	for (i = 0; i < 64; ++i) {
		RND(S[0], S[1], S[2], S[3], S[4], S[5], S[6], S[7], i);
		t = S[7]; S[7] = S[6]; S[6] = S[5]; S[5] = S[4];
		S[4] = S[3]; S[3] = S[2]; S[2] = S[1]; S[1] = S[0]; S[0] = t;
	}

	/* feedback */
	for (i = 0; i < 8; i++) {
		md->state[i] = md->state[i] + S[i];
	}
	lx_crypto_wipe(S,sizeof(S));lx_crypto_wipe(W,sizeof(W));
	return 0;
}


/* Initialize the hash state */
static void sha256_init(struct sha256_state *md)
{
	md->curlen = 0;
	md->length = 0;
	md->state[0] = 0x6A09E667UL;
	md->state[1] = 0xBB67AE85UL;
	md->state[2] = 0x3C6EF372UL;
	md->state[3] = 0xA54FF53AUL;
	md->state[4] = 0x510E527FUL;
	md->state[5] = 0x9B05688CUL;
	md->state[6] = 0x1F83D9ABUL;
	md->state[7] = 0x5BE0CD19UL;
}

/**
   Process a block of memory though the hash
   @param md     The hash state
   @param in     The data to hash
   @param inlen  The length of the data (octets)
   @return CRYPT_OK if successful
*/
static int sha256_process(struct sha256_state *md, const unsigned char *in,
		   unsigned long inlen)
{
	unsigned long n;

	if (md->curlen >= sizeof(md->buf))
		return -1;

	while (inlen > 0) {
		if (md->curlen == 0 && inlen >= SHA256_BLOCK_SIZE) {
			if (sha256_compress(md, (unsigned char *) in) < 0)
				return -1;
			md->length += SHA256_BLOCK_SIZE * 8;
			in += SHA256_BLOCK_SIZE;
			inlen -= SHA256_BLOCK_SIZE;
		} else {
			n = MIN(inlen, (SHA256_BLOCK_SIZE - md->curlen));
			os_memcpy(md->buf + md->curlen, in, n);
			md->curlen += n;
			in += n;
			inlen -= n;
			if (md->curlen == SHA256_BLOCK_SIZE) {
				if (sha256_compress(md, md->buf) < 0)
					return -1;
				md->length += 8 * SHA256_BLOCK_SIZE;
				md->curlen = 0;
			}
		}
	}

	return 0;
}


/**
   Terminate the hash to get the digest
   @param md  The hash state
   @param out [out] The destination of the hash (32 bytes)
   @return CRYPT_OK if successful
*/
static int sha256_done(struct sha256_state *md, unsigned char *out)
{
	int i;

	if (md->curlen >= sizeof(md->buf))
		return -1;

	/* increase the length of the message */
	md->length += md->curlen * 8;

	/* append the '1' bit */
	md->buf[md->curlen++] = (unsigned char) 0x80;

	/* if the length is currently above 56 bytes we append zeros
	 * then compress.  Then we can fall back to padding zeros and length
	 * encoding like normal.
	 */
	if (md->curlen > 56) {
		while (md->curlen < SHA256_BLOCK_SIZE) {
			md->buf[md->curlen++] = (unsigned char) 0;
		}
		sha256_compress(md, md->buf);
		md->curlen = 0;
	}

	/* pad up to 56 bytes of zeroes */
	while (md->curlen < 56) {
		md->buf[md->curlen++] = (unsigned char) 0;
	}

	/* store length */
	WPA_PUT_BE64(md->buf + 56, md->length);
	sha256_compress(md, md->buf);

	/* copy output */
	for (i = 0; i < 8; i++)
		WPA_PUT_BE32(out + (4 * i), md->state[i]);

	return 0;
}

/* ===== end - public domain SHA256 implementation ===== */

void lx_crypto_wipe(void *p, size_t n)
{ volatile uint8_t *v=p; while(n--) *v++=0; }
int lx_crypto_equal(const void *a,const void *b,size_t n)
{ const uint8_t *x=a,*y=b;unsigned v=0;for(size_t i=0;i<n;i++) v|=x[i]^y[i];return v==0; }
void lx_sha256_vector(unsigned count,const uint8_t *const *p,const size_t *n,uint8_t out[32])
{ (void)sha256_vector(count,p,n,out); }
void lx_hmac256(const uint8_t *key,size_t keylen,unsigned count,const uint8_t *const *p,const size_t *n,uint8_t out[32])
{
    struct { struct sha256_state s;uint8_t hashed[32],pad[64],inner[32]; } w;
    if(keylen>64) { const uint8_t *v[]={key};size_t z[]={keylen};lx_sha256_vector(1,v,z,w.hashed);key=w.hashed;keylen=32; }
    for(unsigned i=0;i<64;i++) w.pad[i]=(i<keylen?key[i]:0)^0x36;
    sha256_init(&w.s);sha256_process(&w.s,w.pad,64);
    for(unsigned i=0;i<count;i++) sha256_process(&w.s,p[i],n[i]);
    sha256_done(&w.s,w.inner);
    for(unsigned i=0;i<64;i++) w.pad[i]^=0x36^0x5c;
    sha256_init(&w.s);sha256_process(&w.s,w.pad,64);sha256_process(&w.s,w.inner,32);sha256_done(&w.s,out);
    lx_crypto_wipe(&w,sizeof(w));
}
/* IEEE 802.11 KDF-Hash-Length. Length and counter are little endian; no NUL. */
void lx_kdf256(const uint8_t key[32],const char *label,const uint8_t *context,size_t contextlen,uint8_t *out,size_t len)
{
    uint8_t counter[2],bits[2]={(uint8_t)(len*8),(uint8_t)(len>>5)},hash[32];
    const uint8_t *p[]={counter,(const uint8_t *)label,context,bits};size_t n[]={2,strlen(label),contextlen,2};
    for(unsigned i=1;len;i++) { counter[0]=(uint8_t)i;counter[1]=(uint8_t)(i>>8);lx_hmac256(key,32,4,p,n,hash);
        size_t take=len<32?len:32;memcpy(out,hash,take);out+=take;len-=take; }
    lx_crypto_wipe(hash,sizeof(hash));
}
/* RFC 5869 expand. SAE labels contain no terminating NUL. */
void lx_hkdf256(const uint8_t key[32],const char *label,uint8_t *out,size_t len)
{
    uint8_t t[32],counter=1;const uint8_t *p[]={t,(const uint8_t *)label,&counter};size_t n[]={0,strlen(label),1};
    while(len) { lx_hmac256(key,32,3,p,n,t);size_t take=len<32?len:32;memcpy(out,t,take);len-=take;out+=take;n[0]=32;counter++; }
    lx_crypto_wipe(t,sizeof(t));
}
