/*
 * GlobalData.cpp
 *
 *  Created on: Jul 25, 2026
 *      Author: Tegar
 */

#include "GlobalData.h"

// =========================================================
// [STRUCT PARAMETER PERMANEN (EEPROM)]
// =========================================================
PermanentSettings_t eepromSettings = {0}; // Nilai awal kosong

// =========================================================
// 1 [STRUCT & KONSTANTA]
// =========================================================
const uint8_t ACK_OK = 0x01;

GPIO_PinState LIMIT_X_Pressed = GPIO_PIN_SET;
GPIO_PinState LIMIT_Y_Pressed = GPIO_PIN_SET;
GPIO_PinState LIMIT_Z_Pressed = GPIO_PIN_SET;

// =========================================================
// 2 [OBJEK & PERIPHERAL]
// =========================================================
StepperMotor* motorX = NULL;
StepperMotor* motorY = NULL;
StepperMotor* motorZ = NULL;

// =========================================================
// 3 [BENDERA KONTROL MESIN (STATE MACHINE)]
// =========================================================
volatile bool isNavigating = false;
volatile bool isPaused = false;
volatile bool isAborted = false;
volatile bool ignoreLimitSwitch = false;
volatile bool flagMintaHoming = false;
volatile bool isAutoWeldActive = false;
volatile bool flagManualJog = false;
volatile bool isMachineHomed = false;

// =========================================================
// [RTOS HANDLES]
// =========================================================
SemaphoreHandle_t SinyalPosisiTercapai;
SemaphoreHandle_t Sem_DataDiterima;

// =========================================================
// [Modbus Communication]
// =========================================================
uint16_t Holding_Registers[TOTAL_HOLDING_REGISTERS]	 = {0}; // Alamat 40001
uint16_t Input_Registers[TOTAL_INPUT_REGISTER]  	 = {0};
uint8_t Coils[TOTAL_COILS]              		     = {0};
uint8_t Discrete_Input[TOTAL_DISCRETE_INPUT]  	     = {0};

uint8_t rxChar;
uint8_t rxBuffer[32]; // Diperbesar sedikit untuk Modbus
uint8_t rxIndex = 0;

// =========================================================
// Tracker CAM Zig-Zag
// =========================================================
MesinConfig_t configLas = {0, 0, 0.0f, 0.0f, 0, 0};
uint16_t indexKolomAktual = 0;
uint16_t indexBarisAktual = 0;


