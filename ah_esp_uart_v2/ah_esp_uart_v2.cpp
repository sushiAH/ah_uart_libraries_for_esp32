/**
 * @file ah_esp_uart_v2
 * @brief
 * uart通信でパケットを受取、motor_controller構造体の共有変数に目標値を書き込む,
 */

#include <Arduino.h>
#include <ah_control_table.h>
#include <ah_esp_uart_v2.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "utils.h"

void init_uart(motor_controller* p) {
  xTaskCreate(uart_rx_task,  // タスク関数
              "uart_rx",     // タスク名
              4096,          // スタックサイズ
              (void*)p,      // タスクに渡す引数
              2,             // 優先度
              NULL);         // タスクハンドル
}

/**
 * @brief 受け取ったデータを,table_addrに応じて、共有変数に書き込む
 *
 * @param p
 * @param table_addr enumで定義されたテーブルのアドレス
 * @param data 書き込むデータ
 */
int write_to_control_table(motor_controller* ctrl, uint8_t table_addr,
                           int32_t data) {
  if (table_addr == OPERATING_MODE_ADDR) {
    ctrl->operating_mode = int(data / 1000);
    return 0;
  }

  else if (table_addr == GOAL_POS_ADDR) {
    ctrl->goal_pos_int = data;
    return 0;
  }

  else if (table_addr == GOAL_VEL_ADDR) {
    ctrl->goal_vel_int = data;
    return 0;
  }

  else if (table_addr == POS_P_ADDR) {
    ctrl->POS.PID.kp = data / 1000.00;
    return 0;
  }

  else if (table_addr == POS_I_ADDR) {
    ctrl->POS.PID.ki = data / 1000.00;
    return 0;
  }

  else if (table_addr == POS_D_ADDR) {
    ctrl->POS.PID.kd = data / 1000.00;
    return 0;
  }

  else if (table_addr == VEL_P_ADDR) {
    ctrl->VEL.PID.kp = data / 1000.00;
    return 0;
  }

  else if (table_addr == VEL_I_ADDR) {
    ctrl->VEL.PID.ki = data / 1000.00;
    return 0;
  }

  else if (table_addr == VEL_D_ADDR) {
    ctrl->VEL.PID.kd = data / 1000.00;
    return 0;
  }

  else if (table_addr == GOAL_PWM_ADDR) {
    ctrl->goal_pwm = data;
    return 0;

  } else {
    return table_addr;
  }
}

/**
 * @brief パケットを受信する
 *
 * @param packet_data
 * @param max_packet_size パケットデータの最大サイズ
 * @return err
 */
int receive_packet(uint8_t* packet_data, int max_packet_size) {
  const uint8_t header = 0xAA;
  uint8_t recv_header = 0;
  uint8_t recv_packet_length = 0;
  int temp_read = 0;

  // 読み捨て
  while (Serial.available() > 0) {
    if (Serial.peek() == header) {
      break;
    }
    Serial.read();
  }

  if (Serial.available() < 1) {
    return 1;
  }
  recv_header = Serial.read();  // ヘッダーの受信
  if (recv_header != header) {
    return 1;
  }

  if (Serial.available() < 1) {
    return 1;
  }

  temp_read = Serial.read();

  if (temp_read < 0) {
    return 1;
  }
  recv_packet_length = (uint8_t)temp_read;

  if (recv_packet_length < 2) {
    return 1;
  }

  if (recv_packet_length > max_packet_size) {
    return 1;
  }

  if (Serial.available() < recv_packet_length - 2) {
    return 1;
  }

  packet_data[0] = recv_header;
  packet_data[1] = recv_packet_length;

  for (int i = 2; i < recv_packet_length; i++) {
    packet_data[i] = Serial.read();
  }

  if (calc_checksum(packet_data, recv_packet_length) != 0) {
    return 1;
  }

  return 0;
}

/**
 * @brief packetデータから、targetを取り出す
 *
 * @param packet_data
 * @param target
 * @return err
 */
int read_target_from_packet_data(uint8_t* packet_data, int32_t* target_array) {
  uint8_t recv_packet_length = packet_data[1];

  if (recv_packet_length == 21) {
    target_array[0] = packet_data[7] << 24 | packet_data[6] << 16 |
                      packet_data[5] << 8 | packet_data[4];
    target_array[1] = packet_data[11] << 24 | packet_data[10] << 16 |
                      packet_data[9] << 8 | packet_data[8];
    target_array[2] = packet_data[15] << 24 | packet_data[14] << 16 |
                      packet_data[13] << 8 | packet_data[12];
    target_array[3] = packet_data[19] << 24 | packet_data[18] << 16 |
                      packet_data[17] << 8 | packet_data[16];

    return 0;
  }
  return 1;
}

/**
 * @brief uart freertos task
 *
 * @param pvParameters motor_controller pointer
 */
void uart_rx_task(void* pvParameters) {
  const int recv_period = 5;
  int max_packet_size = 64;
  uint8_t packet_data[max_packet_size] = {0};
  uint8_t table_addr = 0;
  uint8_t packet_length = 0;

  // 書き込み用変数
  int32_t operating_mode = 0;
  uint8_t rx_motor_id = 0;
  int32_t target_array[4] = {0};

  motor_controller* ctrl = (motor_controller*)pvParameters;

  // 周期管理準備
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(recv_period);

  while (1) {
    // パケット受信
    if (receive_packet(packet_data, max_packet_size) == 0) {
      packet_length = packet_data[1];
      rx_motor_id = packet_data[2];
      table_addr = packet_data[3];
      read_target_from_packet_data(packet_data, target_array);
    }

    // 共有変数への書き込み
    if (packet_length == 21) {
      for (int i = 0; i < 4; i++) {
        if (xSemaphoreTake(ctrl[i].mutex, portMAX_DELAY) == pdTRUE) {
          write_to_control_table(&ctrl[i], table_addr, target_array[i]);
          xSemaphoreGive(ctrl[i].mutex);
        }
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
