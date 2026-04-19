#ifndef MODBUS_TEST_CRC_H
#define MODBUS_TEST_CRC_H

#include <stdint.h>

uint16_t CRC16(const uint8_t* data, uint16_t length);

#endif
