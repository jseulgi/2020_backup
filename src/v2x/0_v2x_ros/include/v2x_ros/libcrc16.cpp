#include "libcrc16.h"

CRC16::CRC16(void) {
	initCRC16();
}
void CRC16::initCRC16(void) {
	uint16_t i, j, crc, c;
	for (i=0;i<256;i++) {
		crc = 0;
		c = i;

		for (j=0;j<8;j++) {
			if ((crc ^ c) & 0x0001) crc = (crc >> 1) ^ CRC16_POLY;
			else crc = crc >> 1;
			c = c >> 1;
		}

		crc16_table_[i] = crc;
	}
	is_crc16_init_ = true;
}
uint16_t CRC16::calcCRC16(const unsigned char *input_str, size_t num_bytes) {
	uint16_t crc;
	const unsigned char *ptr;
	size_t a;

	if (!is_crc16_init_) initCRC16();

	crc = CRC16_START;
	ptr = input_str;

	if (ptr != NULL) for (a=0;a<num_bytes;a++) {
		crc = (crc >> 8) ^ crc16_table_[(crc ^ (uint16_t) *ptr++) & 0x00FF];
	}

	return crc;
}
