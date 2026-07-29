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
const float STEPS_PER_MM = 160.0f; // Sesuaikan dengan microstep & pitch mekanik Anda
const uint8_t ACK_OK = 0x01;

GPIO_PinState LIMIT_X_Pressed = GPIO_PIN_SET;
GPIO_PinState LIMIT_Y_Pressed = GPIO_PIN_SET;

volatile float X_MAX_SPEED = 20833.0f;
volatile float Y_MAX_SPEED = 20833.0f;

volatile float X_MIN_SPEED = 3000.0f;
volatile float Y_MIN_SPEED = 3000.0f;

volatile float X_HOMING_MAX_SPEED = 6000.0f;
volatile float Y_HOMING_MAX_SPEED = 6000.0f;

// =========================================================
// 2 [OBJEK & PERIPHERAL]
// =========================================================
StepperMotor* motorX = NULL;
StepperMotor* motorY = NULL;

volatile uint16_t JarakXSetelahHoming = 3200;
volatile uint16_t JarakYSetelahHoming = 3200;

// =========================================================
// 3 [BENDERA KONTROL MESIN (STATE MACHINE)]
// =========================================================
volatile bool isNavigating = false;
volatile bool isPaused = false;
volatile bool isAborted = false;
volatile bool ignoreLimitSwitch = false;
volatile bool flagMintaHoming = false;
volatile bool isAutoWeldActive = false;
volatile bool flagManualWeld = false;

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
// Tracker CAM Zig-Zag
// =========================================================
MesinConfig_t configLas = {0, 0, 0.0f, 0.0f};
uint16_t indexKolomAktual = 0;
uint16_t indexBarisAktual = 0;


