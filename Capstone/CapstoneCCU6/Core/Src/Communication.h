#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// Fungsi Task & Kalkulasi Modbus
void Task_Komunikasi(void *pvParameters);


void Function_Code_04(uint8_t* rxBuffer, uint8_t len);
void Function_Code_05(uint8_t* rxBuffer, uint8_t len);
void Function_Code_06(uint8_t* rxBuffer, uint8_t len);

uint16_t Hitung_CRC16(const uint8_t *nData, uint16_t wLength);
void Process_System_Reset(uint8_t* rxBuffer, uint16_t len);
void Send_Modbus_Exception(uint8_t slave_id, uint8_t function_code, uint8_t exception_code);

uint16_t Hitung_CRC16(const uint8_t *nData, uint16_t wLength);

#ifdef __cplusplus
}
#endif

#endif // COMMUNICATION_H
