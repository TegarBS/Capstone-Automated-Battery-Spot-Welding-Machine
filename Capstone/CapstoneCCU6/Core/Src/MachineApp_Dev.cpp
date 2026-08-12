/*
 * MachineApp_Dev.cpp
 *
 *  Created on: Jul 25, 2026
 *      Author: Tegar
 */

#include "MachineApp.h"
#include "StepperMotor.h"
#include "main.h"
#include "Communication.h"
#include "GlobalData.h"
#include "UtilitasDanPerhitungan.h"
#include "FungsiEEPROM.h"
#include "Monitor.h"



void Task_Sequencer(void *pvParameters);

bool Wait_Motors_Reach_Target(bool includeZ);
bool Delay_With_Watchdog(uint32_t total_delay_ms);
void Handle_Emergency_Abort(void);
void Perform_Homing_Sequence(void);
void Perform_Auto_Weld_Cycle(void);
void Perform_Manual_Jog(void);


// Definisi Pin Fiktif untuk Relay dan Las
#ifndef RELAY_Z_PORT
#define RELAY_Z_PORT GPIOB
#define RELAY_Z_PIN GPIO_PIN_10
#define WELD_PORT GPIOB
#define WELD_PIN GPIO_PIN_11
#endif

// --- FUNGSI INISIALISASI MESIN ---
void MachineApp_Init(void) {
	// -----------------------------------------------------
	// DIAGNOSTIK PENYEBAB RESET (PRE-FLIGHT CHECK)
	// -----------------------------------------------------
	// 1. Cek apakah mesin baru saja di-reset oleh Watchdog
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
		kode_alarm_sistem = 1; // Set alarm IWDG!
	}
	// 2. Cek apakah mesin di-reset oleh tombol fisik (Pin Reset)
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
		kode_alarm_sistem = 2;
	}
	// 3. Cek apakah mesin baru saja dicolok listrik (Power On Reset)
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
		kode_alarm_sistem = 3;
	}

	// WAJIB DILAKUKAN: Hapus bendera reset di memori hardware,
	// jika tidak, status IWDG akan terus terbaca di reset berikutnya!
	__HAL_RCC_CLEAR_RESET_FLAGS();

	Load_Settings_From_Flash();

    motorX = new StepperMotor(STEP_X_GPIO_Port, STEP_X_Pin, DIR_X_GPIO_Port, DIR_X_Pin, &htim2);
    motorY = new StepperMotor(STEP_Y_GPIO_Port, STEP_Y_Pin, DIR_Y_GPIO_Port, DIR_Y_Pin, &htim3);
    motorZ = new StepperMotor(STEP_Z_GPIO_Port, STEP_Z_Pin, DIR_Z_GPIO_Port, DIR_Z_Pin, &htim4);

    // Buat Semaphore & Queue
    SinyalPosisiTercapai = xSemaphoreCreateBinary();
    Sem_DataDiterima = xSemaphoreCreateBinary();

    // Buat Task RTOS
    xTaskCreate(Task_Sequencer, "Task_Seq", 512, NULL, 2, NULL);
    xTaskCreate(Task_Komunikasi, "Task_Komunikasi", 512, NULL, 2, NULL);
    xTaskCreate(Task_Monitor, "Task_Mon", 256, NULL, 1, NULL);

    Update_Motor_Param_From_Settings();


}

void MachineApp_Loop(void) {}

void Task_Sequencer(void *pvParameters) {
	Delay_With_Watchdog(500); // Jeda inisialisasi hardware

    while(1) {
        if (isAborted) {
            Handle_Emergency_Abort();
        }
        else if (flagMintaHoming) {
            Perform_Homing_Sequence();
        }
        else if (isAutoWeldActive) {
            Perform_Auto_Weld_Cycle();
        }
        else if (flagManualJog) {
            Perform_Manual_Jog();
        }
        else {
            // =========================================================
            // E. MODE STANDBY PURE (Menunggu Perintah)
            // =========================================================
            vTaskDelay(pdMS_TO_TICKS(50));
            heartbeat_Sequencer = true; // ABSENSI WAJIB
        }
    }
}



// ---------------------------------------------------------
// HELPER 1: Menunggu Motor Selesai Bergerak (Aman dari Watchdog & E-Stop)
// Return TRUE jika sukses sampai tujuan, FALSE jika dibatalkan (E-Stop)
// ---------------------------------------------------------
bool Wait_Motors_Reach_Target(bool includeZ = false) {
    while (motorX->isMoving() || motorY->isMoving() || (includeZ && motorZ->isMoving())) {
        xSemaphoreTake(SinyalPosisiTercapai, pdMS_TO_TICKS(20));
        heartbeat_Sequencer = true; // LAPOR MANDOR IWDG!

        if (isAborted) return false; // Tembus seketika jika E-Stop ditekan
    }
    return true;
}

void Handle_Emergency_Abort(void) {
    isAutoWeldActive = false;
    isAborted = false;
    isMachineHomed = false;

    HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);

    motorX->stop();
    motorY->stop();
    motorZ->stop();
}

void Perform_Homing_Sequence(void) {
    Update_Motor_Param_Homing_From_Settings();

    // --- TAHAP 1: SEEK ---
    isNavigating = true;
    motorX->isHomed = false;
    motorY->isHomed = false;

    if (HAL_GPIO_ReadPin(LIMIT_X_GPIO_Port, LIMIT_X_Pin) == LIMIT_X_Pressed) motorX->stop();
    else motorX->setTargetPulse(-999999);

    if (HAL_GPIO_ReadPin(LIMIT_Y_GPIO_Port, LIMIT_Y_Pin) == LIMIT_Y_Pressed) motorY->stop();
    else motorY->setTargetPulse(-999999);

    if (!Wait_Motors_Reach_Target()) {
		isNavigating = false;
		flagMintaHoming = false; // Batalkan niat homing
		return; // Keluar dari fungsi ini SEKARANG JUGA!
	} // Gagal / Batal

    motorX->resetPosition();
    motorY->resetPosition();

    // --- TAHAP 2: PULL-OFF ---
    ignoreLimitSwitch = true;
    motorX->setTargetPulse(Convert_mm_to_PulseX(eepromSettings.homingTravelAfterLimitX));
    motorY->setTargetPulse(Convert_mm_to_PulseY(eepromSettings.homingTravelAfterLimitY));

    if (!Wait_Motors_Reach_Target()) {
		isNavigating = false;
		flagMintaHoming = false; // Batalkan niat homing
		return; // Keluar dari fungsi ini SEKARANG JUGA!
	}

    ignoreLimitSwitch = false;

    // --- TAHAP 3: ZERO ABSOLUT ---
    motorX->resetPosition();
    motorY->resetPosition();
    Update_Motor_Param_From_Settings();

    isMachineHomed = true;
    flagMintaHoming = false;
}

void Perform_Auto_Weld_Cycle(void) {
	if (isFirstWeldCycle) {
		if (configLas.startKolom > configLas.totalKolom || configLas.startBaris > configLas.totalBaris) {
			indexKolomAktual = 0;
			indexBarisAktual = 0;
		} else {
			indexKolomAktual = configLas.startKolom;
			indexBarisAktual = configLas.startBaris;
		}

		isFirstWeldCycle = false; // GEMBOK PINTUNYA!
	}

    if (!isMachineHomed) {
        flagMintaHoming = true; return;
    }

    if (indexBarisAktual >= configLas.totalBaris) {
        isAutoWeldActive = false; return;
    }

    while (isPaused && !isAborted) {
        heartbeat_Sequencer = true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (isAborted) return;

    Koordinat_t center = Calculate_Battery_Center_mm();

    // MINI-LOOP
    for (uint16_t i = 0; i < eepromSettings.weldCount; i++) {
        float offsetY_mm = 0.0f;
        if (eepromSettings.weldCount > 1) {
            float spacing = (float)eepromSettings.spotSpacing_x10 / 10.0f;
            offsetY_mm = -(spacing * (eepromSettings.weldCount - 1) / 2.0f) + (i * spacing);
        }

        isNavigating = true;
        motorX->setTargetPulse(Convert_mm_to_PulseX(center.x));
        motorY->setTargetPulse(Convert_mm_to_PulseY(center.y + offsetY_mm));

        if (!Wait_Motors_Reach_Target()) return; // Gagal / Batal
        isNavigating = false;

//        // --- Eksekusi Las dengan Helper Delay ---
//        HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_SET);
//        if (!Delay_With_Watchdog(Holding_Registers[4])) return; // Drop Delay
//
//        HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_SET);
//        vTaskDelay(pdMS_TO_TICKS(Holding_Registers[5])); // Weld Time
//        HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);
//
//        HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET);
//        if (!Delay_With_Watchdog(200)) return; // Pneumatic Rise Delay
        Delay_With_Watchdog(500);
    }

	// 5. UPDATE INDEX BATERAI SELANJUTNYA (ZIG-ZAG FISIK)
	if (indexBarisAktual % 2 == 0) {
		// ---------------------------------------------------
		// BARIS GENAP (0, 2, 4...): Mesin bergerak Kiri -> Kanan
		// ---------------------------------------------------
		if (indexKolomAktual < configLas.totalKolom - 1) {
			indexKolomAktual++; // Lanjut bergeser ke kanan
		} else {
			// Sudah mentok di ujung KANAN
			indexBarisAktual++; // Pindah baris (turun ke sumbu Y)
			// PENTING: indexKolomAktual TIDAK di-nol-kan. Ia tetap di ujung kanan!
		}

	} else {

		// ---------------------------------------------------
		// BARIS GANJIL (1, 3, 5...): Mesin bergerak Kanan -> Kiri
		// ---------------------------------------------------
		if (indexKolomAktual > 0) {
			indexKolomAktual--; // Lanjut bergeser ke kiri (Mundur)
		} else {
			// Sudah mentok di ujung KIRI
			indexBarisAktual++; // Pindah baris (turun ke sumbu Y)
			// PENTING: indexKolomAktual sudah bernilai 0 (ujung kiri). Biarkan saja!
		}

	}
}

void Perform_Manual_Jog(void) {
    long jogManX_mm = (long)( ((uint32_t)Holding_Registers[7] << 16) | (uint32_t)Holding_Registers[6] );
    long jogManY_mm = (long)( ((uint32_t)Holding_Registers[9] << 16) | (uint32_t)Holding_Registers[8] );
    long jogManZ_mm = (long)( ((uint32_t)Holding_Registers[11] << 16) | (uint32_t)Holding_Registers[10] );

    long targetAbsX = motorX->getCurrentPositionPulse() + Convert_mm_to_PulseX(jogManX_mm);
    long targetAbsY = motorY->getCurrentPositionPulse() + Convert_mm_to_PulseY(jogManY_mm);
    long targetAbsZ = motorZ->getCurrentPositionPulse() + Convert_mm_to_PulseZ(jogManZ_mm);

    isNavigating = true;
    motorX->setTargetPulse(targetAbsX);
    motorY->setTargetPulse(targetAbsY);
    motorZ->setTargetPulse(targetAbsZ);

    Wait_Motors_Reach_Target(true); // Termasuk motor Z

    isNavigating = false;
    flagManualJog = false;

    // Reset Register
    for(int i = 6; i <= 11; i++) Holding_Registers[i] = 0;
}
