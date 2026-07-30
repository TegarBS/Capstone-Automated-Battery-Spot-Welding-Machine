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
// [STRUCT PARAMETER PERMANEN (EEPROM)]
// =========================================================
typedef struct {
    uint32_t magicNumber; // Penanda apakah Flash sudah pernah diisi atau masih kosong
    uint16_t stepsPerMmX;
    uint16_t stepsPerMmY;
    uint16_t maxSpeed;
    uint16_t accelSpeed;

    uint16_t STEP_PER_MM_X;
    uint16_t STEP_PER_MM_Y;
    uint16_t STEP_PER_MM_Z;

    uint16_t SPEED_MAX_X;
    uint16_t SPEED_MAX_Y;
    uint16_t SPEED_MAX_Z;

    uint16_t SPEED_MIN_X;
    uint16_t SPEED_MIN_Y;
    uint16_t SPEED_MIN_Z;

    uint16_t ACC_X;
    uint16_t ACC_Y;
    uint16_t ACC_Z;

    uint16_t TRAVEL_MAX_X;
    uint16_t TRAVEL_MAX_Y;
    uint16_t TRAVEL_MAX_Z;

    uint16_t HOMING_SPEED_MAX_X;
    uint16_t HOMING_SPEED_MAX_Y;
    uint16_t HOMING_SPEED_MAX_Z;

    uint16_t HOMING_TRAVEL_AFTER_LIMIT_X;
    uint16_t HOMING_TRAVEL_AFTER_LIMIT_Y;
    uint16_t HOMING_TRAVEL_AFTER_LIMIT_Z;

    uint16_t DIRECTION_X;
    uint16_t DIRECTION_Y;
    uint16_t DIRECTION_Z;

} PermanentSettings_t;

extern PermanentSettings_t eepromSettings;

// =========================================================
// 1 [STRUCT & KONSTANTA]
// =========================================================
typedef struct {
    long x;
    long y;
} Koordinat_t;

typedef struct {
    uint16_t totalKolom;
    uint16_t totalBaris;
    float pitchX;
    float pitchY;
} MesinConfig_t;

extern const float STEPS_PER_MM;
extern const uint8_t ACK_OK;

extern GPIO_PinState LIMIT_X_Pressed;
extern GPIO_PinState LIMIT_Y_Pressed;

extern float X_MAX_SPEED;
extern float Y_MAX_SPEED;

extern float X_MIN_SPEED;
extern float Y_MIN_SPEED;

extern float X_HOMING_MAX_SPEED;
extern float Y_HOMING_MAX_SPEED;

extern uint16_t JarakXSetelahHoming;
extern uint16_t JarakYSetelahHoming;

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
extern volatile bool flagMintaHoming;
extern volatile bool isAutoWeldActive;
extern volatile bool flagManualWeld;

// =========================================================
// [RTOS HANDLES]
// =========================================================
extern SemaphoreHandle_t SinyalPosisiTercapai;
extern SemaphoreHandle_t Sem_DataDiterima;

// =========================================================
// [Modbus Communication]
// =========================================================
extern uint16_t Holding_Registers[40]; // Alamat 40001 - 40040
extern uint16_t Input_Registers[10];
extern uint8_t Coils[10];
extern uint8_t Discrete_Input[10];

extern uint8_t rxChar;
extern uint8_t rxBuffer[32]; // Diperbesar sedikit untuk Modbus
extern uint8_t rxIndex;

// =========================================================
// [TRACKER CAM ZIG-ZAG]
// =========================================================
extern MesinConfig_t configLas;
extern uint16_t indexKolomAktual;
extern uint16_t indexBarisAktual;

// =========================================================
// Pointer Hardware dari CubeMX (main.c)
// =========================================================
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#endif /* SRC_GLOBALDATA_H_ */
