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

#define TOTAL_HOLDING_REGISTERS 70
#define TOTAL_INPUT_REGISTER 10
#define TOTAL_COILS 10
#define TOTAL_DISCRETE_INPUT 10

#define EEPROM_START_MODBUS_REG 20
#define TOTAL_EEPROM_PARAMS 28

#define SKALA_FLOAT 100.0f

// =========================================================
// [STRUCT PARAMETER PERMANEN (EEPROM)]
// =========================================================
typedef struct __attribute__((aligned(4))){
    uint32_t magicNumber; // Penanda apakah Flash sudah pernah diisi atau masih kosong

    uint16_t stepsPerMmX;
    uint16_t stepsPerMmY;
    uint16_t stepsPerMmZ;

    //Speed in mm/s
    uint16_t maxSpeedX;
    uint16_t maxSpeedY;
    uint16_t maxSpeedZ;

    uint16_t minSpeedX;
    uint16_t minSpeedY;
    uint16_t minSpeedZ;

    //Acceleration in mm/s^2
    uint16_t accelX;
    uint16_t accelY;
    uint16_t accelZ;

    //Max travel in mm
    uint16_t maxTravelX;
    uint16_t maxTravelY;
    uint16_t maxTravelZ;

    //Speed in mm/s
    uint16_t maxSpeedHomingX;
    uint16_t maxSpeedHomingY;
    uint16_t maxSpeedHomingZ;

    //Travel in mm
    uint16_t homingTravelAfterLimitX;
    uint16_t homingTravelAfterLimitY;
    uint16_t homingTravelAfterLimitZ;

    uint16_t directionX;
    uint16_t directionY;
    uint16_t directionZ;

    uint16_t weldCount;
    uint16_t spotSpacing_x10; //Jika ingin jarak 2.0mm, data dikirim bernilai 20

    uint16_t X_distance_from_0;
    uint16_t Y_distance_from_0;

} PermanentSettings_t;

extern PermanentSettings_t eepromSettings;



// =========================================================
// 1 [STRUCT & KONSTANTA]
// =========================================================
typedef struct {
    float x;
    float y;
} Koordinat_t;

typedef struct {
    uint16_t totalKolom;
    uint16_t totalBaris;
    float pitchX;
    float pitchY;
    uint16_t startKolom;       // Reg 6 (Index awal X)
    uint16_t startBaris;
} MesinConfig_t;

extern const uint8_t ACK_OK;

extern GPIO_PinState LIMIT_X_Pressed;
extern GPIO_PinState LIMIT_Y_Pressed;
extern GPIO_PinState LIMIT_Z_Pressed;

extern uint16_t kode_alarm_sistem;

// =========================================================
// 2 [OBJEK & PERIPHERAL]
// =========================================================
extern StepperMotor* motorX;
extern StepperMotor* motorY;
extern StepperMotor* motorZ;

// =========================================================
// 3 [BENDERA KONTROL MESIN (STATE MACHINE)]
// =========================================================
extern volatile bool isNavigating;
extern volatile bool isPaused;
extern volatile bool isAborted;
extern volatile bool ignoreLimitSwitch;
extern volatile bool flagMintaHoming;
extern volatile bool isAutoWeldActive;
extern volatile bool flagManualJog;
extern volatile bool isMachineHomed;
extern volatile bool isFirstWeldCycle;

// =========================================================
// [RTOS HANDLES]
// =========================================================
extern SemaphoreHandle_t SinyalPosisiTercapai;
extern SemaphoreHandle_t Sem_DataDiterima;

// =========================================================
// [Modbus Communication]
// =========================================================
extern uint16_t Holding_Registers[TOTAL_HOLDING_REGISTERS]; // Alamat 40001 - 40040
extern uint16_t Input_Registers[TOTAL_INPUT_REGISTER];
extern uint8_t Coils[TOTAL_COILS];
extern uint8_t Discrete_Input[TOTAL_DISCRETE_INPUT];

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
extern TIM_HandleTypeDef htim4;

extern IWDG_HandleTypeDef hiwdg;

// =========================================================
// Variabel "Detak Jantung" (Heartbeat) untuk setiap Task
// =========================================================
extern volatile bool heartbeat_Sequencer;
extern volatile bool heartbeat_Komunikasi;

#endif /* SRC_GLOBALDATA_H_ */
