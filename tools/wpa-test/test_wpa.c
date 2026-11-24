/*
 * Tests of wifi/lx/src/lx_wpa.c on the machine the kernel is built on.
 *
 * Two kinds. The pieces against the examples their own standards give (they are quoted here with
 * where they are from). And whole handshakes against an access point written in Python on
 * implementations that share nothing with ours (reference.py: hashlib, hmac, the "cryptography"
 * package's AES key wrap): what a correct station answers is known there to the byte.
 */
#include <stdio.h>
#include <stdlib.h>

#include "lx_wpa.c"

static unsigned checks, failed;

static void
check(int good, const char *what, int n)
{
	checks++;
	if (!good) {
		failed++;
		printf("FAILED: %s (%d)\n", what, n);
	}
}

static int
hex_is(const unsigned char *bytes, unsigned length, const char *hex)
{
	for (unsigned i = 0; i < length; i++) {
		unsigned value;

		if (sscanf(hex + 2 * i, "%2x", &value) != 1 || value != bytes[i]) {
			return 0;
		}
	}
	return hex[2 * length] == 0;
}

struct handshake {
	const unsigned char *passphrase; size_t passphrase_length;
	const unsigned char *name; size_t name_length;
	const unsigned char *ap; size_t ap_length;
	const unsigned char *station; size_t station_length;
	const unsigned char *snonce; size_t snonce_length;
	const unsigned char *rsn; size_t rsn_length;
	const unsigned char *pmk; size_t pmk_length;
	const unsigned char *tk; size_t tk_length;
	const unsigned char *group; size_t group_length;
	const unsigned char *group2; size_t group2_length;
	const unsigned char *m1; size_t m1_length;
	const unsigned char *m2; size_t m2_length;
	const unsigned char *m3; size_t m3_length;
	const unsigned char *m4; size_t m4_length;
	const unsigned char *g1; size_t g1_length;
	const unsigned char *g2; size_t g2_length;
	const unsigned char *m3_wrong; size_t m3_wrong_length;
	const unsigned char *igtk; size_t igtk_length;
	const unsigned char *igtk2; size_t igtk2_length;
	unsigned group_index, pmf, sae, owe;
};

struct bip_case { const unsigned char *key, *frame; unsigned length; };
#include "cases.inc"

static void
pieces(void)
{
	unsigned char digest[20], pmk[32], out[80], block[16], round_keys[176];
	struct sha1 s;

	/* FIPS 180-4, the two examples of SHA-1 */
	sha1_begin(&s);
	sha1_more(&s, (const unsigned char *)"abc", 3);
	sha1_end(&s, digest);
	check(hex_is(digest, 20, "a9993e364706816aba3e25717850c26c9cd0d89d"), "SHA-1 of \"abc\"", 0);
	sha1_begin(&s);
	sha1_more(&s, (const unsigned char *)"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq", 56);
	sha1_end(&s, digest);
	check(hex_is(digest, 20, "84983e441c3bd26ebaae4aa1f95129e5e54670f1"), "SHA-1 of the 56-byte example", 0);

	/* RFC 2202, HMAC-SHA1 test cases 1, 2 and 6 (a key longer than a block) */
	{
		unsigned char key[80];
		const unsigned char *one[1];
		unsigned length[1];

		memset(key, 0x0b, 20);
		one[0] = (const unsigned char *)"Hi There";
		length[0] = 8;
		hmac_sha1(key, 20, one, length, 1, digest);
		check(hex_is(digest, 20, "b617318655057264e28bc0b6fb378c8ef146be00"), "HMAC-SHA1, RFC 2202 case 1", 0);
		one[0] = (const unsigned char *)"what do ya want for nothing?";
		length[0] = 28;
		hmac_sha1((const unsigned char *)"Jefe", 4, one, length, 1, digest);
		check(hex_is(digest, 20, "effcdf6ae5eb2fa2d27416d5f184df9c259a7c79"), "HMAC-SHA1, RFC 2202 case 2", 0);
		memset(key, 0xaa, 80);
		one[0] = (const unsigned char *)"Test Using Larger Than Block-Size Key - Hash Key First";
		length[0] = 54;
		hmac_sha1(key, 80, one, length, 1, digest);
		check(hex_is(digest, 20, "aa4ae5e15272d00e95705637ce8a3b55ed402112"), "HMAC-SHA1, RFC 2202 case 6", 0);
	}

	/* IEEE 802.11-2020, J.4.2: the three examples of a passphrase's key */
	lx_wpa_pmk((const unsigned char *)"password", 8, (const unsigned char *)"IEEE", 4, pmk);
	check(hex_is(pmk, 32, "f42c6fc52df0ebef9ebb4b90b38a5f902e83fe1b135a70e23aed762e9710a12e"), "PMK, J.4.2 example 1", 0);
	lx_wpa_pmk((const unsigned char *)"ThisIsAPassword", 15, (const unsigned char *)"ThisIsASSID", 11, pmk);
	check(hex_is(pmk, 32, "0dc0d6eb90555ed6419756b9a15ec3e3209b63df707dd508d14581f8982721af"), "PMK, J.4.2 example 2", 0);
	lx_wpa_pmk((const unsigned char *)"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 32, (const unsigned char *)"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ", 32,
	    pmk);
	check(hex_is(pmk, 32, "becb93866bb8c3832cb777c2f559807c8c59afcb6eae734885001300a981cc62"), "PMK, J.4.2 example 3", 0);

	/* FIPS 197, C.1: the example of AES-128, backwards */
	{
		static const unsigned char key[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
		static const unsigned char cipher[16] = { 0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4,
			                                  0xc5, 0x5a };
		/* RFC 3394, 4.1: 128 bits of key data wrapped with a 128-bit key */
		static const unsigned char wrapped[24] = { 0x1f, 0xa6, 0x8b, 0x0a, 0x81, 0x12, 0xb4, 0x47, 0xae, 0xf3, 0x4b, 0xd8, 0xfb,
			                                   0x5a, 0x7b, 0x82, 0x9d, 0x3e, 0x86, 0x23, 0x71, 0xd2, 0xcf, 0xe5 };
		unsigned char spoiled[24];

		aes_keys(key, round_keys);
		check(hex_is(round_keys + 160, 16, "13111d7fe3944a17f307a78b4d2b30c5"), "AES-128 key schedule, FIPS 197 C.1: the last "
		    "round key", 0);
		memcpy(block, cipher, 16);
		aes_decrypt(round_keys, block);
		check(hex_is(block, 16, "00112233445566778899aabbccddeeff"), "AES-128 decryption, FIPS 197 C.1", 0);
		check(aes_unwrap(key, wrapped, 24, out) == 0 && hex_is(out, 16, "00112233445566778899aabbccddeeff"),
		    "AES key unwrap, RFC 3394 4.1", 0);
		memcpy(spoiled, wrapped, 24);
		spoiled[17] ^= 1;
		check(aes_unwrap(key, spoiled, 24, out) != 0, "AES key unwrap of spoiled data is refused", 0);
		check(aes_unwrap(key, wrapped, 20, out) != 0, "AES key unwrap of a wrong length is refused", 0);
	}

	/* the PRF and a longer unwrap against the independent implementations */
	{
		unsigned char data[70], plain[40];

		for (unsigned i = 0; i < 70; i++) {
			data[i] = (unsigned char)i;
		}
		prf(prf_key, sizeof(prf_key), "A label", data, 70, out, 61);
		check(memcmp(out, prf_out, 61) == 0, "the PRF, 61 bytes, against Python's hmac", 0);
		check(aes_unwrap(wrap_kek, wrap_out, sizeof(wrap_out), plain) == 0 && plain[0] == 100 && plain[39] == 139,
		    "AES key unwrap of 40 bytes against \"cryptography\"", 0);
	}
}

static void
group_keys(void)
{
	unsigned char kde[40] = { 0xdd, 38, 0, 0x0f, 0xac, 1, 1, 0 };
	const unsigned char counter[8] = { 1, 2, 3, 4, 5, 6, 0, 0 };
	struct lx_wpa wpa = { 0 }, saved;

	for (unsigned i = 0; i < 32; i++) {
		kde[8 + i] = (unsigned char)i;
	}
	wpa.group_cipher = 2;
	check(group_key(&wpa, kde, sizeof(kde), counter) == 0 && wpa.group_length == 32 &&
	    hex_is(wpa.group, 32, "000102030405060708090a0b0c0d0e0f18191a1b1c1d1e1f1011121314151617"),
	    "TKIP keeps TK and swaps the two Michael MIC halves", 0);
	saved = wpa;
	for (unsigned length = 0; length < sizeof(kde); length++) {
		check(group_key(&wpa, kde, length, counter) == -1 && memcmp(&wpa, &saved, sizeof(wpa)) == 0,
		    "truncated TKIP KDE leaves the previous key untouched", (int)length);
	}
	kde[1] = 22;
	check(group_key(&wpa, kde, 24, counter) == -1, "16-byte GTK refused for negotiated TKIP", 0);
	wpa.group_cipher = 4;
	kde[1] = 38;
	check(group_key(&wpa, kde, sizeof(kde), counter) == -1, "32-byte GTK refused for negotiated CCMP", 0);
	wpa.group_cipher = 2;
	kde[6] = 0;
	check(group_key(&wpa, kde, sizeof(kde), counter) == -1, "GTK cannot use the pairwise key index", 0);
	kde[6] = 3;
	check(group_key(&wpa, kde, sizeof(kde), counter) == 0 && wpa.group_index == 3 &&
	    memcmp(wpa.group_counter, counter, sizeof(counter)) == 0, "TKIP index 3 and replay sequence retained", 0);
	wpa.group_cipher = 8;
	check(group_key(&wpa, kde, sizeof(kde), counter) == -1, "unknown negotiated group cipher refused", 0);
}

static void
whole(void)
{
	for (unsigned n = 0; n < sizeof(handshakes) / sizeof(handshakes[0]); n++) {
		const struct handshake *h = &handshakes[n];
		unsigned char reply[LX_WPA_REPLY_MOST], spoiled[MESSAGE_MOST];
		unsigned length = 0;
		const char *why = NULL;
		struct lx_wpa wpa;

		memset(&wpa, 0, sizeof(wpa));
		if (h->sae || h->owe) memcpy(wpa.pmk,h->pmk,32);
		else lx_wpa_pmk(h->passphrase, (unsigned)h->passphrase_length, h->name, (unsigned)h->name_length, wpa.pmk);
		check(memcmp(wpa.pmk, h->pmk, 32) == 0, "the PMK against Python's pbkdf2_hmac", (int)n);
		memcpy(wpa.own, h->station, 6);
		memcpy(wpa.peer, h->ap, 6);
		memcpy(wpa.own_nonce, h->snonce, 32);
		wpa.rsn = h->rsn;
		wpa.rsn_length = (unsigned)h->rsn_length;
		wpa.group_cipher = h->rsn[7];
		wpa.pmf = h->pmf;
		wpa.sae = h->sae;
		wpa.owe = h->owe; wpa.owe_ready = 1;
		wpa.ap_rsn = h->rsn;
		wpa.ap_rsn_length = h->rsn[1]+2;
        if(h->rsn_length>wpa.ap_rsn_length) {
            wpa.ap_rsnxe=h->rsn+wpa.ap_rsn_length;
            wpa.ap_rsnxe_length=(unsigned)h->rsn_length-wpa.ap_rsn_length;
        }

        if(h->owe) {
            wpa.owe_ready=0;
            check(lx_wpa_message(&wpa,h->m1,(unsigned)h->m1_length,reply,&length,&why)==LX_WPA_NOTHING &&
                !length && !wpa.have_keys,"OWE early M1 waits for the PMK",(int)n);
            wpa.owe_ready=1;
        }
		/* a signed message before any message 1 */
		check(lx_wpa_message(&wpa, h->m3, (unsigned)h->m3_length, reply, &length, &why) == LX_WPA_BAD, "message 3 first is refused",
		    (int)n);
		check(lx_wpa_message(&wpa, h->m1, (unsigned)h->m1_length, reply, &length, &why) == LX_WPA_ANSWER &&
		    length == h->m2_length && memcmp(reply, h->m2, length) == 0, "message 2 is the reference's", (int)n);
		check(memcmp(wpa.pairwise, h->tk, 16) == 0, "the pairwise key is the reference's", (int)n);

		/* message 3 from an access point with another passphrase, and one with a bit turned */
		check(lx_wpa_message(&wpa, h->m3_wrong, (unsigned)h->m3_wrong_length, reply, &length, &why) == LX_WPA_BAD &&
		    length == 0, "message 3 under another passphrase is refused", (int)n);
		memcpy(spoiled, h->m3, h->m3_length);
		spoiled[AT_DATA + 3] ^= 0x40;
		check(lx_wpa_message(&wpa, spoiled, (unsigned)h->m3_length, reply, &length, &why) == LX_WPA_BAD,
		    "message 3 with a bit turned is refused", (int)n);

		check(lx_wpa_message(&wpa, h->m3, (unsigned)h->m3_length, reply, &length, &why) == LX_WPA_KEYS &&
		    length == h->m4_length && memcmp(reply, h->m4, length) == 0, "message 4 is the reference's", (int)n);
		check(wpa.group_length == h->group_length && memcmp(wpa.group, h->group, h->group_length) == 0 && wpa.group_index == h->group_index,
		    "the group key and its number are the reference's", (int)n);
		check(memcmp(wpa.group_counter, h->m3 + AT_RSC, 8) == 0, "the group key's packet number is message 3's", (int)n);

		check(!h->pmf || (wpa.igtk[0].valid && memcmp(wpa.igtk[0].key, h->igtk, 16) == 0), "PMF message 3 installs IGTK 4", (int)n);

		/* message 3 once more: played again */
		check(lx_wpa_message(&wpa, h->m3, (unsigned)h->m3_length, reply, &length, &why) == LX_WPA_BAD,
		    "message 3 played again is refused", (int)n);

		/* a new group key later */
		check(lx_wpa_message(&wpa, h->g1, (unsigned)h->g1_length, reply, &length, &why) == LX_WPA_GROUP &&
		    length == h->g2_length && memcmp(reply, h->g2, length) == 0, "the answer to a new group key is the reference's",
		    (int)n);
		check(wpa.group_length == h->group2_length && memcmp(wpa.group, h->group2, h->group2_length) == 0 && wpa.group_index == h->group_index % 3 + 1,
		    "the new group key and its number are the reference's", (int)n);

		check(!h->pmf || (wpa.igtk[1].valid && memcmp(wpa.igtk[1].key, h->igtk2, 16) == 0 && wpa.igtk[0].valid), "PMF group rekey installs IGTK 5 and retains 4", (int)n);

		/* a network that wants another kind of key management says so in the descriptor's version */
		memcpy(spoiled, h->m1, h->m1_length);
		spoiled[AT_INFO + 1] = (unsigned char)((spoiled[AT_INFO + 1] & ~7) | 3);
		check(lx_wpa_message(&wpa, spoiled, (unsigned)h->m1_length, reply, &length, &why) == LX_WPA_BAD && why[0] != 0,
		    "a key descriptor of version 3 is refused, with a reason", (int)n);

		/* things that are not key messages, and ones cut short */
		check(lx_wpa_message(&wpa, (const unsigned char *)"\x02\x00\x00\x00", 4, reply, &length, &why) == LX_WPA_NOTHING,
		    "an EAPOL frame that is not a key message is left alone", (int)n);
		check(lx_wpa_message(&wpa, h->m1, 60, reply, &length, &why) == LX_WPA_BAD, "a key message cut short is refused", (int)n);
	}
}

static void
pmf_tests(void)
{
    /* RFC 4493 section 4, all four lengths (empty, full and partial final blocks). */
    const uint8_t key[16] = {0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c};
    const uint8_t msg[64] = {
        0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
        0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
        0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
        0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10};
    const unsigned sizes[] = {0,16,40,64};
    const char *tags[] = {"bb1d6929e95937287fa37d129b756746", "070a16b46b4d4144f79bdd9dd04a287c",
        "dfa66747de9ae63030ca32611497c827", "51f0bebf7e3b9d92fc49741779363cfe"};
    for (unsigned i=0;i<4;i++) {
        uint8_t out[16];
        const uint8_t *parts[] = {msg};
        cmac(key,parts,&sizes[i],1,out);
        check(hex_is(out,16,tags[i]),"RFC 4493 CMAC vector",(int)i);
    }
    for (unsigned i=0;i<sizeof(bip_cases)/sizeof(bip_cases[0]);i++) {
        const struct bip_case *c=&bip_cases[i];
        struct lx_wpa w={0}, before;
        unsigned slot=i%2;
        w.pmf=1;
        w.igtk[slot].valid=1;
        memcpy(w.igtk[slot].key,c->key,16);
        before=w;
        for (unsigned bit=0;bit<c->length*8;bit++) {
            uint8_t f[128]; unsigned n=c->length;
            w=before;
            memcpy(f,c->frame,n); f[bit/8]^=1u<<(bit%8);
            /* Duration, sequence number, and three FC flags are outside BIP's authentication. */
            if (bit/8==2 || bit/8==3 || bit/8==22 || bit/8==23 || (bit/8==1 && (bit%8>=3 && bit%8<=5))) continue;
            /* Destination bit 0 can turn this into unicast: not firmware-decrypted, still refused. */
            check(lx_wpa_pmf_rx(&w,f,&n,0,0,0,0)==-1 && memcmp(&w,&before,sizeof(w))==0,
                "BIP rejects tampering without advancing IPN",(int)bit);
        }
        for (unsigned n=0;n<c->length;n++) {
            uint8_t f[128]; unsigned len=n;
            w=before; memcpy(f,c->frame,c->length);
            check(lx_wpa_pmf_rx(&w,f,&len,0,0,0,0)==-1,"truncated BIP refused",(int)n);
        }
        uint8_t f[128]; unsigned n=c->length;
        w=before; memcpy(f,c->frame,n);
        check(lx_wpa_pmf_rx(&w,f,&n,0,0,0,0)==0 && n==26,"reference BIP accepted and MMIE removed",(int)i);
        memcpy(f,c->frame,c->length); n=c->length;
        check(lx_wpa_pmf_rx(&w,f,&n,0,0,0,0)==-1,"BIP replay refused",(int)i);
    }
    /* Transactional KDE parsing, required IGTK, both slots and reinstall IPN preservation. */
    {
        struct lx_wpa w={0}, saved;
        uint8_t kde[54]={0xdd,22,0,15,172,1,1,0}, counter[8]={0};
        for(unsigned i=0;i<16;i++) kde[8+i]=i;
        memcpy(kde+24,"\xdd\x1c\x00\x0f\xac\x09\x04\x00",8);
        kde[32]=1;
        memset(kde+38,0x5a,16);
        w.pmf=1; w.group_cipher=4; saved=w;
        check(key_data(&w,kde,24,counter,1)==-1 && memcmp(&w,&saved,sizeof(w))==0,"initial PMF requires IGTK",0);
        for(unsigned n=25;n<54;n++) {
            check(key_data(&w,kde,n,counter,1)==-1 && memcmp(&w,&saved,sizeof(w))==0,"truncated IGTK is atomic",(int)n);
        }
        check(key_data(&w,kde,54,counter,1)==0 && w.group_updated && w.igtk[0].valid,"GTK and IGTK committed together",0);
        w.igtk[0].ipn[0]=50;
        check(key_data(&w,kde+24,30,counter,0)==0 && !w.group_updated && w.igtk[0].ipn[0]==50,"IGTK-only rekey cannot roll IPN back",0);
        saved=w; kde[30]=3;
        check(key_data(&w,kde,54,counter,1)==-1 && memcmp(&w,&saved,sizeof(w))==0,"invalid IGTK index cannot change GTK",0);
        kde[30]=4; kde[25]=27;
        check(key_data(&w,kde,54,counter,1)==-1,"wrong IGTK size refused",0);
        kde[25]=28;
        uint8_t dup[84]; memcpy(dup,kde,54); memcpy(dup+54,kde+24,30);
        check(key_data(&w,dup,84,counter,1)==-1,"duplicate IGTK refused",0);
        kde[30]=5;
        check(key_data(&w,kde+24,30,counter,0)==0 && w.igtk[1].ipn[0]==50,"same key in other slot keeps IPN",0);
    }
    /* Message 3 authenticates the AP's advertised RSN, including its PMF capabilities. */
    {
        uint8_t rsn[22]={48,20,1,0,0,15,172,4,1,0,0,15,172,4,1,0,0,15,172,2,0x80,0};
        uint8_t data[46]; memcpy(data,rsn,22);
        memcpy(data+22,"\xdd\x16\x00\x0f\xac\x01\x01\x00",8); memset(data+30,0x5a,16);
        struct lx_wpa w={0}, saved; uint8_t counter[8]={0};
        w.ap_rsn=rsn; w.ap_rsn_length=22; saved=w;
        data[20]=0;
        check(key_data(&w,data,46,counter,1)==-1 && memcmp(&w,&saved,sizeof(w))==0,"RSN downgrade in message 3 rejected atomically",0);
        check(key_data(&w,data+22,24,counter,1)==-1,"message 3 must authenticate the AP RSN",0);
        data[20]=0x80;
        check(key_data(&w,data,46,counter,1)==0,"matching AP RSN accepted",0);
    }
    /* Firmware-decrypted unicast: CCMP header/MIC removal and independent management PN. */
    for (unsigned stripped=0;stripped<2;stripped++) {
        struct lx_wpa w={0};
        uint8_t original[42]={0xc0,0x40};
        original[4]=2; original[24]=1; original[27]=0x20; original[32]=3;
        uint8_t f[42]; unsigned n=sizeof(f)-(stripped?8:0);
        memcpy(f,original,sizeof(f));
        check(lx_wpa_pmf_rx(&w,f,&n,0,0,stripped,0)==-1,"unverified unicast rejected",0);
        check(lx_wpa_pmf_rx(&w,f,&n,1,0,stripped,0)==0 && n==26 && f[24]==3,"decrypted CCMP normalized",0);
        memcpy(f,original,sizeof(f)); n=sizeof(f)-(stripped?8:0);
        check(lx_wpa_pmf_rx(&w,f,&n,1,0,stripped,0)==-1,"unicast management replay refused",0);
        n=26;
        check(lx_wpa_pmf_rx(&w,f,&n,1,1,1,0)==-1,"stripped PN requires validation",0);
        check(lx_wpa_pmf_rx(&w,f,&n,1,1,1,1)==0,"driver-validated stripped PN accepted",0);
    }
}

int
main(void)
{
	pieces();
	group_keys();
	whole();
	pmf_tests();
	printf("%u checks, %u failed\n", checks, failed);
	return failed != 0;
}
