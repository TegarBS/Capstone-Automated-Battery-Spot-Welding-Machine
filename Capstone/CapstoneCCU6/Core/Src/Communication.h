#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// Fungsi Task & Kalkulasi Modbus
void Task_Komunikasi(void *pvParameters);
uint16_t Hitung_CRC16(const uint8_t *nData, uint16_t wLength);

#ifdef __cplusplus
}
#endif

#endif // COMMUNICATION_H
