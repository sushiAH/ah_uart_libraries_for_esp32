#include <utils.h>

void reset_array(uint8_t* array, int length) {
  for (int i = 0; i < length; i++) {
    array[i] = 0;
  }
}

/**
 * @brief 配列の長さ分排他的論理和を取る
 *
 * @param array
 * @param length 配列長さ
 * @return 排他的論理和
 */
uint8_t sum(uint8_t* array, int length) {
  uint8_t sum_value = 0;
  for (int i = 0; i < length; i++) {
    sum_value ^= array[i];
  }
  return sum_value;
}

/**
 * @brief チェックサムを実行する
 *
 * @param recv_data 受け取ったデータ
 * @param length データ長さ
 * @return bool
 */
bool calc_checksum(uint8_t* array, int length) {
  uint8_t sum_value = 0;
  sum_value = sum(array, length);

  if (sum_value == 0) {
    return 0;

  } else {
    return 1;
  }
}

/**
 * @brief int32を4つのバイト列に変換
 *
 * @param data
 * @param split_data
 */
void from_int32_to_bytes(int32_t data, uint8_t* split_data) {
  split_data[3] = (uint8_t)(((data) >> 24) & 0xFF);
  split_data[2] = (uint8_t)(((data) >> 16) & 0xFF);
  split_data[1] = (uint8_t)(((data) >> 8) & 0xFF);
  split_data[0] = (uint8_t)((data) & 0xFF);
}
