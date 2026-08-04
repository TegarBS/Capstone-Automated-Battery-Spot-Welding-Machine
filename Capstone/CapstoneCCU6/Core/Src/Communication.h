#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// Fungsi Task & Kalkulasi Modbus
void Task_Komunikasi(void *pvParameters);

void Function_Code_01_READ_COILS(uint8_t* rxBufferInput, uint8_t len);
bool Baca_Status_Coil(uint16_t coil_addr) ;

void Function_Code_02_READ_DISCRETE_INPUT(uint8_t* rxBufferInput, uint8_t len);
bool Baca_Discrete_Input(uint16_t input_addr);

void Function_Code_03_READ_HOLDING_REGISTER(uint8_t* rxBufferInput, uint8_t len);

void Function_Code_04_READ_INPUT_REGISTER(uint8_t* rxBufferInput, uint8_t len);

void Function_Code_05_WRITE_SINGLE_COIL(uint8_t* rxBufferInput, uint8_t len);

void Function_Code_06_WRITE_SINGLE_REGISTER(uint8_t* rxBufferInput, uint8_t len);

uint16_t Hitung_CRC16(const uint8_t *nData, uint16_t wLength);
void Process_System_Reset(uint8_t* rxBuffer, uint16_t len);
void Send_Modbus_Exception(uint8_t slave_id, uint8_t function_code, uint8_t exception_code);

uint16_t Hitung_CRC16(const uint8_t *nData, uint16_t wLength);

#ifdef __cplusplus
}
#endif

#endif // COMMUNICATION_H
