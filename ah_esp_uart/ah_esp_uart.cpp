/**
 * @file ah_esp_uart.cpp
 * @brief esp_uartライブラリ
 */

#include <Arduino.h>
#include <ah_control_table.h>
#include <ah_esp_uart.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "utils.h"

void init_uart(motor_controller* ctrl) {
  xTaskCreate(uart_serial,  // タスク関数
              "uart",       // タスク名
              4096,         // スタックサイズ
              (void*)ctrl,  // タスクに渡す引数
              2,            // 優先度
              NULL);        // タスクハンドル
}

/**
 * @brief 受け取ったデータを,table_addrに応じて、共有変数に書き込む
 *
 * @param ctrl
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
    return 0;
    ctrl->goal_vel_int = data;
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
    return table_addr;  // 未定義のアドレス
  }
}

/**
 * @brief table_addrに応じて、読み出した現在値を返す
 *
 * @param ctrl
 * @param table_addr コントロールテーブルアドレス
 * @param motor_id モーターID
 * @return 現在値
 */
int32_t read_current_data(motor_controller* ctrl, uint8_t table_addr,
                          uint8_t motor_id) {
  if (table_addr == CURRENT_POS_ADDR) {
    return ctrl->current_pos_int;
  }

  else if (table_addr == CURRENT_SPEED_ADDR) {
    return ctrl->current_vel_int;
  }

  return 0;  // 未定義のアドレス
}

/**
 * @brief パケットを送信する
 *
 * @param motor_id モーターid
 * @param send_data 送信するデータ
 */
void send_packet(int motor_id, int32_t send_data) {
  const uint8_t header = 0xAA;
  const uint8_t packet_length = 8;

  uint8_t packet[8] = {0};
  uint8_t split_data[4] = {0};

  from_int32_to_bytes(send_data, split_data);

  packet[0] = header;
  packet[1] = packet_length;
  packet[2] = (uint8_t)motor_id;
  packet[3] = split_data[0];
  packet[4] = split_data[1];
  packet[5] = split_data[2];
  packet[6] = split_data[3];
  packet[7] = sum(packet, packet_length - 1);

  Serial.write(packet, packet_length);
}

/**
 * @brief パケットを受信する
 *
 * @param packet_data
 * @param max_packet_size　パケット最大サイズ
 * @return err
 */
int receive_packet(uint8_t* packet_data, int max_packet_size) {
  const uint8_t header = 0xAA;

  // headerの受信
  while (Serial.available() > 0) {
    // 読み飛ばし
    if (Serial.peek() == header) {
      break;
    }
    Serial.read();
  }

  uint8_t recv_header = Serial.read();
  if (recv_header != header) {
    return 1;
  }

  // packet_length受信
  uint8_t recv_packet_length = Serial.read();

  if (recv_packet_length < 2 || recv_packet_length > max_packet_size) {
    return 1;
  }

  if (Serial.available() < recv_packet_length - 2) {
    return 1;
  }

  packet_data[0] = recv_header;
  packet_data[1] = recv_packet_length;

  // 残りのデータ受信
  for (int i = 2; i < recv_packet_length; i++) {
    packet_data[i] = Serial.read();
  }

  // チェックサム
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
int read_target_from_packet_data(uint8_t* packet_data, int32_t* target) {
  uint8_t recv_packet_length = packet_data[1];

  if (recv_packet_length == 6) {
    *target = packet_data[4];
    return 0;
  }

  if (recv_packet_length == 9) {
    *target = packet_data[7] << 24 | packet_data[6] << 16 |
              packet_data[5] << 8 | packet_data[4];
    return 0;
  }

  return 1;
}

/**
 * @brief uart freertos task
 *
 * @param pvParameters motor_controller pointer
 */
void uart_serial(void* pvParameters) {
  const int recv_period = 5;
  const int max_packet_size = 256;

  uint8_t packet_data[max_packet_size] = {0};
  uint8_t table_addr = 0;
  uint8_t packet_length = 0;

  // 書き込み用変数
  int32_t operating_mode = 0;
  int32_t target = 0;
  int32_t current_data = 0;
  uint8_t rx_motor_id = 0;

  motor_controller* ctrl = (motor_controller*)pvParameters;

  // 周期管理
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(recv_period);

  while (1) {
    // パケット受信

    if (receive_packet(packet_data, max_packet_size) == 0) {
      packet_length = packet_data[1];
      rx_motor_id = packet_data[2];
      table_addr = packet_data[3];
      read_target_from_packet_data(packet_data, &target);
    }

    if (xSemaphoreTake(ctrl[rx_motor_id].mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      // 共有変数へ書き込み
      write_to_control_table(&ctrl[rx_motor_id], table_addr, target);

      // 共有変数から読み出し
      current_data =
          read_current_data(&ctrl[rx_motor_id], table_addr, rx_motor_id);
      xSemaphoreGive(ctrl[rx_motor_id].mutex);
    }

    // 現在値を送信
    if (table_addr == CURRENT_POS_ADDR || table_addr == CURRENT_SPEED_ADDR) {
      send_packet(rx_motor_id, current_data);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
