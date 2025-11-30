#include <Arduino.h>
#include <ah_control_table.h>
#include <ah_esp_uart.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"

void init_uart(motor_controller* p) {
  xTaskCreate(uart_serial,  // タスク関数
              "uart",       // タスク名
              4096,         // スタックサイズ
              (void*)p,     // タスクに渡す引数
              2,            // 優先度
              NULL);        // タスクハンドル
}

void write_to_control_table(motor_controller* p, uint8_t table_addr,
                            int32_t data) {
  if (table_addr == OPERATING_MODE_ADDR) {
    p->operating_mode = data;
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

int read_current_data(motor_controller* p, uint8_t table_addr,
                      uint8_t motor_id) {
  int32_t current_data = 0;

  if (table_addr == CURRENT_POS_ADDR) {
    current_data = p->current_pos_int;

  } else if (table_addr == CURRENT_SPEED_ADDR) {
    current_data = p->current_vel_int;
  }

  return current_data;
}

void reset_array(uint8_t* array, int length) {
  for (int i = 0; i < length; i++) {
    array[i] = 0;
  }
}

int sum(uint8_t* array, int length) {
  int sum_value = 0;
  for (int i = 0; i < length; i++) {
    sum_value = sum_value + array[i];
  }
  return sum_value;
}

int check_sum(uint8_t* recv_data, int length) {
  int sum_value = sum(recv_data, length - 1);
  if ((sum_value % 256) == (recv_data[length - 1] % 256)) {
    return 0;
  } else {
    return 1;
  }
}

void send_packet(int motor_id, int32_t send_data) {
  uint8_t packet_length = 8;
  uint8_t packet[8] = {0};
  uint8_t header = 0xAA;
  int32_t send_data_integer = 0;
  uint8_t split_data[4] = {0};

  send_data_integer = send_data;
  from_int32_to_bytes(send_data_integer, split_data);

  packet[0] = header;
  packet[1] = packet_length;
  packet[2] = (uint8_t)motor_id;
  packet[3] = split_data[0];
  packet[4] = split_data[1];
  packet[5] = split_data[2];
  packet[6] = split_data[3];
  packet[7] = sum(packet, packet_length - 1) % 256;

  Serial.write(packet, packet_length);
}

void from_int32_to_bytes(int32_t data, uint8_t* split_data) {
  split_data[3] = (uint8_t)(((data) >> 24) & 0xFF);
  split_data[2] = (uint8_t)(((data) >> 16) & 0xFF);
  split_data[1] = (uint8_t)(((data) >> 8) & 0xFF);
  split_data[0] = (uint8_t)((data) & 0xFF);
}

int receive_packet(uint8_t* packet_data, int max_packet_size) {
  const uint8_t header = 0xAA;
  uint8_t recv_header = 0;
  uint8_t recv_packet_length = 0;

  int temp_read = 0;

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

int get_target_from_packet_data(uint8_t* packet_data, int32_t* target) {
  uint8_t recv_packet_length = 0;
  recv_packet_length = packet_data[1];

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

void uart_serial(void* pvParameters) {
  const int recv_period = 5;
  int max_packet_size = 256;
  uint8_t packet_data[max_packet_size] = {0};
  uint8_t table_addr = 0;
  uint8_t packet_length = 0;

  int32_t operating_mode = 0;
  uint8_t rx_motor_id = 0;
  int32_t target = 0;
  int32_t current_data = 0;

  motor_controller* p = (motor_controller*)pvParameters;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(recv_period);

  while (1) {
    if (receive_packet(packet_data, max_packet_size) == 0) {
      packet_length = packet_data[1];
      rx_motor_id = packet_data[2];
      table_addr = packet_data[3];
      get_target_from_packet_data(packet_data, &target);
    }

    if (xSemaphoreTake(p[rx_motor_id].mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      write_to_control_table(&p[rx_motor_id], table_addr, target);
      current_data =
          read_current_data(&p[rx_motor_id], table_addr, rx_motor_id);
      xSemaphoreGive(p[rx_motor_id].mutex);
    }

    if (table_addr == CURRENT_POS_ADDR || table_addr == CURRENT_SPEED_ADDR) {
      send_packet(rx_motor_id, current_data);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
