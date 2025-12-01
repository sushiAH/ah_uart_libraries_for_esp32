#ifndef AH_ESP_UART_V2_H
#define AH_ESP_UART_V2_H

#include <Arduino.h>
#include <ah_pid_esp.h>
#include <stdint.h>

void init_uart(motor_controller* ctrl);

int receive_packet(uint8_t* packet_data, int max_buffer_size);

int read_target_from_packet_data(uint8_t* packet_data, int32_t* target);

void send_packet(int motor_id, int32_t send_data);

int write_to_control_table(motor_controller* ctrl, uint8_t table_addr,
                           int32_t data);

void uart_rx_task(void* pvParameters);

#endif
