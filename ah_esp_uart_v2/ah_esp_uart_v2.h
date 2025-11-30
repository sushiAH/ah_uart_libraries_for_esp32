#ifndef AH_ESP_UART_V2_H
#define AH_ESP_UART_V2_H

#include <Arduino.h>
#include <stdint.h>
#include <ah_pid_esp.h>

//パケットを受け取る
int receive_packet(uint8_t* packet_data,int max_buffer_size);

int get_target_from_packet_data(uint8_t* packet_data,int32_t* target);

void from_int32_to_bytes(int32_t data,uint8_t* splited_data);

void send_packet(int motor_id,int32_t send_data);

void write_to_control_table(motor_controller* p,uint8_t table_addr,int32_t data);

void init_uart(motor_controller* p);

int sum(uint8_t* array,int length);

int reset_array(uint8_t* array,int length);

int check_sum(uint8_t* recv_data,int data_length);

void uart_rx_task(void* pvParameters);

void uart_tx_task(void* pvParameters);


  


#endif
