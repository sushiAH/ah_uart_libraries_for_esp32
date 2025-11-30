#include <Arduino.h>
#include <ah_control_table.h>
#include <ah_esp_uart_v2.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"

void init_uart(motor_controller* p) {
  xTaskCreate(uart_rx_task,  // タスク関数
              "uart_rx",     // タスク名
              4096,          // スタックサイズ
              (void*)p,      // タスクに渡す引数
              2,             // 優先度
              NULL);         // タスクハンドル
}

// motor_controller内の共有変数に書き込む
void write_to_control_table(motor_controller* p, uint8_t table_addr,
                            int32_t data) {
  if (table_addr == OPERATING_MODE_ADDR) {
    p->operating_mode = int(data / 1000);
  }

  else if (table_addr == GOAL_POS_ADDR) {
    p->goal_pos_int = data;
  }

  else if (table_addr == GOAL_VEL_ADDR) {
    p->goal_vel_int = data;
  }

  else if (table_addr == POS_P_ADDR) {
    p->POS.PID.kp = data / 1000.00;
  }

  else if (table_addr == POS_I_ADDR) {
    p->POS.PID.ki = data / 1000.00;
  }

  else if (table_addr == POS_D_ADDR) {
    p->POS.PID.kd = data / 1000.00;
  }

  else if (table_addr == VEL_P_ADDR) {
    p->VEL.PID.kp = data / 1000.00;
  }

  else if (table_addr == VEL_I_ADDR) {
    p->VEL.PID.ki = data / 1000.00;
  }

  else if (table_addr == VEL_D_ADDR) {
    p->VEL.PID.kd = data / 1000.00;
  }

  else if (table_addr == GOAL_PWM_ADDR) {
    p->goal_pwm = data;
  }
}

// 配列を0に初期化する
int reset_array(uint8_t* array, int length) {
  for (int i = 0; i < length; i++) {
    array[i] = 0;
  }
}

// 配列の要素全てを足し合わせる
int sum(uint8_t* array, int length) {
  int sum_value = 0;
  for (int i = 0; i < length; i++) {
    sum_value = sum_value + array[i];
  }
  return sum_value;
}

// チェックサムを実行する
int check_sum(uint8_t* recv_data, int length) {
  int sum_value = sum(recv_data, length - 1);
  if ((sum_value % 256) == (recv_data[length - 1] % 256)) {
    return 0;
  } else {
    return 1;
  }
}

// パケットを送信する
void send_packet(int motor_id, int32_t send_data) {
  uint8_t packet_length = 8;
  uint8_t packet[8] = {0};
  uint8_t header = 0xAA;
  int32_t send_data_integer = 0;
  uint8_t splited_data[4] = {0};

  send_data_integer = send_data;
  from_int32_to_bytes(send_data_integer, splited_data);

  packet[0] = header;
  packet[1] = packet_length;
  packet[2] = (uint8_t)motor_id;
  packet[3] = splited_data[0];  // リトルエンディアン
  packet[4] = splited_data[1];
  packet[5] = splited_data[2];
  packet[6] = splited_data[3];
  packet[7] = sum(packet, packet_length - 1) % 256;

  // send
  Serial.write(packet, packet_length);
}

// int32を4つのbyteに分割する
// 下位バイトから格納するリトルエンディアン
void from_int32_to_bytes(int32_t data, uint8_t* splited_data) {
  splited_data[3] = (uint8_t)(((data) >> 24) & 0xFF);
  splited_data[2] = (uint8_t)(((data) >> 16) & 0xFF);
  splited_data[1] = (uint8_t)(((data) >> 8) & 0xFF);
  splited_data[0] = (uint8_t)((data) & 0xFF);
}

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

  if (check_sum(packet_data, recv_packet_length) != 0) {
    return 1;
  }

  return 0;
}

int get_target_from_packet_data(uint8_t* packet_data, int32_t* target_array) {
  uint8_t recv_packet_length = 0;
  recv_packet_length = packet_data[1];

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

  // 読み出し用変数　
  int32_t current_pos = 0;
  int32_t current_vel = 0;

  motor_controller* p = (motor_controller*)pvParameters;

  // 周期管理準備
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(recv_period);

  while (1) {
    // 受信
    if (receive_packet(packet_data, max_packet_size) == 0) {
      packet_length = packet_data[1];
      rx_motor_id = packet_data[2];
      table_addr = packet_data[3];
      get_target_from_packet_data(packet_data, target_array);
    }

    if (packet_length == 21) {
      for (int i = 0; i < 4; i++) {
        if (xSemaphoreTake(p[i].mutex, portMAX_DELAY) == pdTRUE) {
          // 共有変数への書き込み
          write_to_control_table(&p[i], table_addr, target_array[i]);
          xSemaphoreGive(p[i].mutex);
        }
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
