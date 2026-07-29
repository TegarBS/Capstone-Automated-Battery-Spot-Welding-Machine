/*
 * GlobalData.h
 *
 *  Created on: Jul 25, 2026
 *      Author: Tegar
 */

#ifndef SRC_GLOBALDATA_H_
#define SRC_GLOBALDATA_H_

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "StepperMotor.h"

// =========================================================
// 1 [STRUCT & KONSTANTA]
// =========================================================
typedef struct {
    long x;
    long y;
} Koordinat_t;

extern GPIO_PinState LIMIT_X_Pressed;
extern GPIO_PinState LIMIT_Y_Pressed;

extern volatile float X_MAX_SPEED;
extern volatile float Y_MAX_SPEED;

extern volatile float X_MIN_SPEED;
extern volatile float Y_MIN_SPEED;

extern volatile float X_HOMING_MAX_SPEED;
extern volatile float Y_HOMING_MAX_SPEED;

// =========================================================
// 2 [OBJEK & PERIPHERAL]
// =========================================================
extern StepperMotor* motorX;
extern StepperMotor* motorY;

// =========================================================
// 3 [BENDERA KONTROL MESIN (STATE MACHINE)]
// =========================================================
extern volatile bool isNavigating;
extern volatile bool isPaused;
extern volatile bool isAborted;
extern volatile bool ignoreLimitSwitch;
extern volatile bool flagHoming;

// =========================================================
// [RTOS HANDLES]
// =========================================================
extern SemaphoreHandle_t SinyalPosisiTercapai;
extern SemaphoreHandle_t Sem_DataDiterima;

// =========================================================
// [Modbus Communication]
// =========================================================
extern uint16_t Holding_Registers[10]; // Alamat 40001 - 40010
extern uint16_t Input_Registers[10];
extern uint8_t Coils[10];

extern uint8_t rxChar;
extern uint8_t rxBuffer[32]; // Diperbesar sedikit untuk Modbus
extern uint8_t rxIndex;

// =========================================================
// Queue
// =========================================================
extern QueueHandle_t Queue_Koordinat;

// =========================================================
// Pointer Hardware dari CubeMX (main.c)
// =========================================================
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#endif /* SRC_GLOBALDATA_H_ */
