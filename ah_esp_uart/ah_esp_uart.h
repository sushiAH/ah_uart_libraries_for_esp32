#ifndef AH_ESP_UART_H
#define AH_ESP_UART_H

#include <Arduino.h>
#include <ah_pid_esp.h>
#include <stdint.h>

// パケットを受け取る
int receive_packet(uint8_t* packet_data, int max_packet_size);

int read_target_from_packet_data(uint8_t* packet_data, int32_t* target);

int32_t read_current_data(motor_controller* ctrl, uint8_t table_addr);

void send_packet(int motor_id, int32_t send_data);

int write_to_control_table(motor_controller* ctrl, uint8_t table_addr,
                           int32_t data);

void send_current_data(motor_controller* ctrl, uint8_t table_addr,
                       uint8_t motor_id);

void uart_serial(void* pvParameters);

void init_uart(motor_controller* ctrl);

#endif
