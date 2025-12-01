#ifndef UTILS_H
#define UTILS_H

#include <cstdint>

void reset_array(uint8_t *array, int length);

uint8_t sum(uint8_t *array, int length);

bool calc_checksum(uint8_t *array, int length);

void from_int32_to_bytes(int32_t data, uint8_t *split_data);

#endif
