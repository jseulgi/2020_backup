#include <iostream>

#define CRC16_POLY 0xA001
#define CRC16_START 0x0000

class CRC16 {
private:
	bool is_crc16_init_;
	int16_t crc16_table_[256];
public:
	CRC16(void);
	void initCRC16(void);
	uint16_t calcCRC16(const unsigned char *input_str, size_t num_bytes);
};
