/*
 * GlobalData.cpp
 *
 *  Created on: Jul 25, 2026
 *      Author: Tegar
 */

#include "GlobalData.h"

// =========================================================
// 1 [STRUCT & KONSTANTA]
// =========================================================


// =========================================================
// 2 [OBJEK & PERIPHERAL]
// =========================================================
StepperMotor* motorX = NULL;
StepperMotor* motorY = NULL;

// =========================================================
// 3 [BENDERA KONTROL MESIN (STATE MACHINE)]
// =========================================================
volatile bool isNavigating = false;
volatile bool isPaused = false;
volatile bool isAborted = false;
volatile bool ignoreLimitSwitch = false;

// =========================================================
// [RTOS HANDLES]
// =========================================================
SemaphoreHandle_t SinyalPosisiTercapai;
SemaphoreHandle_t Sem_DataDiterima;

// =========================================================
// [Modbus Communication]
// =========================================================
uint16_t Holding_Registers[10] = {0}; // Alamat 40001 - 40010
uint16_t Input_Registers[10]   = {0};
uint8_t Coils[10]              = {0};

uint8_t rxChar;
uint8_t rxBuffer[32]; // Diperbesar sedikit untuk Modbus
uint8_t rxIndex = 0;

// =========================================================
// Queue
// =========================================================
QueueHandle_t Queue_Koordinat;


