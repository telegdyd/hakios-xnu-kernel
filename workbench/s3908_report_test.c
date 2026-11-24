#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "board/s3908_report.h"

/* bits into a payload, least significant first (as the chip packs them) */
static void
put(struct s3908_tcm *tcm, uint32_t *offset, uint32_t bits, uint32_t value)
{
	for (uint32_t i = 0; i < bits; i++, (*offset)++) {
		if (value >> i & 1) {
			tcm->payload[*offset / 8] |= (uint8_t)(1u << (*offset % 8));
		}
	}
}

int
main(void)
{
	static struct s3908_tcm tcm;
	/* a config of the OPLUS driver's kind: a timestamp, the count, then per active object index, class, x and y */
	static const uint8_t config[] = {
		S3908_TOUCH_TIMESTAMP, 32,
		S3908_TOUCH_NUM_OF_ACTIVE_OBJECTS, 8,
		S3908_TOUCH_FOREACH_ACTIVE_OBJECT,
		S3908_TOUCH_OBJECT_N_INDEX, 4,
		S3908_TOUCH_OBJECT_N_CLASSIFICATION, 4,
		S3908_TOUCH_OBJECT_N_X_POSITION, 16,
		S3908_TOUCH_OBJECT_N_Y_POSITION, 16,
		S3908_TOUCH_OBJECT_N_Z, 8,
		S3908_TOUCH_FOREACH_END,
		S3908_TOUCH_END,
	};
	uint32_t offset = 0;

	memcpy(tcm.config, config, sizeof(config));
	tcm.config_length = sizeof(config);

	put(&tcm, &offset, 32, 123456);
	put(&tcm, &offset, 8, 2);
	put(&tcm, &offset, 4, 3);
	put(&tcm, &offset, 4, 1);
	put(&tcm, &offset, 16, 5760);
	put(&tcm, &offset, 16, 100);
	put(&tcm, &offset, 8, 40);
	put(&tcm, &offset, 4, 7);
	put(&tcm, &offset, 4, 1);
	put(&tcm, &offset, 16, 2880);
	put(&tcm, &offset, 16, 12864);
	put(&tcm, &offset, 8, 40);
	tcm.payload_length = (offset + 7) / 8;
	assert(s3908_parse_touch(&tcm) == 0);
	assert(tcm.object[3].status == 1 && tcm.object[3].x == 5760 && tcm.object[3].y == 100);
	assert(tcm.object[7].status == 1 && tcm.object[7].x == 2880 && tcm.object[7].y == 12864);
	assert(tcm.object[0].status == 0 && tcm.object[9].status == 0);

	/* no finger: the count is 0, and the old objects are cleared */
	memset(tcm.payload, 0, sizeof(tcm.payload));
	offset = 0;
	put(&tcm, &offset, 32, 1);
	put(&tcm, &offset, 8, 0);
	tcm.payload_length = (offset + 7) / 8;
	assert(s3908_parse_touch(&tcm) == 0);
	for (unsigned i = 0; i < S3908_MAX_OBJECTS; i++) {
		assert(tcm.object[i].status == 0);
	}

	/* an index past the objects: not a report */
	memset(tcm.payload, 0, sizeof(tcm.payload));
	offset = 0;
	put(&tcm, &offset, 32, 1);
	put(&tcm, &offset, 8, 1);
	put(&tcm, &offset, 4, 12);
	tcm.payload_length = 10;
	assert(s3908_parse_touch(&tcm) < 0);

	/* a config cut off after a code that wants its width */
	tcm.config_length = 1;
	tcm.config[0] = S3908_TOUCH_TIMESTAMP;
	assert(s3908_parse_touch(&tcm) < 0);

	/* bits across bytes */
	memset(tcm.payload, 0, sizeof(tcm.payload));
	tcm.payload[0] = 0xf0;
	tcm.payload[1] = 0x0f;
	tcm.payload_length = 2;
	assert(s3908_bits(&tcm, 4, 8) == 0xff && s3908_bits(&tcm, 12, 8) == 0 && s3908_bits(&tcm, 0, 33) == 0);

	puts("S3908 report: two fingers, none, a bad index, a cut config and bits across bytes checked");
	return 0;
}
