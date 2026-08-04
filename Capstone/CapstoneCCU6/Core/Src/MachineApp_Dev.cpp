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

// Definisi Pin Fiktif untuk Relay dan Las
#ifndef RELAY_Z_PORT
#define RELAY_Z_PORT GPIOB
#define RELAY_Z_PIN GPIO_PIN_10
#define WELD_PORT GPIOB
#define WELD_PIN GPIO_PIN_11
#endif

void Task_Sequencer(void *pvParameters);


// --- FUNGSI INISIALISASI MESIN ---
void MachineApp_Init(void) {
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

    // MENGHIDUPKAN JANTUNG MESIN
    HAL_UART_Receive_IT(&huart1, &rxChar, 1);
}

void MachineApp_Loop(void) {}

void Task_Sequencer(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(500)); // Jeda inisialisasi hardware

    while(1) {
        // =========================================================
        // A. KONDISI DARURAT (ABORT / E-STOP)
        // =========================================================
        if (isAborted) {
            isAutoWeldActive = false; // Batalkan siklus otomatis
            isAborted = false;        // Reset bendera
            isMachineHomed = false;

            HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET); // Naikkan pneumatik
            HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);       // Matikan trafo

            // [HARDWARE ISR SEHARUSNYA SUDAH MEMATIKAN INI, TAPI KITA DOUBLE CHECK DI SINI]
            motorX->stop();
            motorY->stop();
            motorZ->stop();
        }

        // =========================================================
        // B. FITUR HOMING MANDIRI (Pre-Flight Check + Pull Off)
        // =========================================================
        if (flagMintaHoming && !isAborted) {

            Update_Motor_Param_Homing_From_Settings();

            // --- TAHAP 1: SEEK (Mencari Limit Switch) ---
            isNavigating = true;
            motorX->isHomed = false;
            motorY->isHomed = false;

            if (HAL_GPIO_ReadPin(LIMIT_X_GPIO_Port, LIMIT_X_Pin) == LIMIT_X_Pressed) {
                motorX->stop();
            } else {
                motorX->setTargetPulse(-999999);
            }

            if (HAL_GPIO_ReadPin(LIMIT_Y_GPIO_Port, LIMIT_Y_Pin) == LIMIT_Y_Pressed) {
                motorY->stop();
            } else {
                motorY->setTargetPulse(-999999);
            }

            // MENGHINDARI portMAX_DELAY: Tunggu dengan batas waktu (20ms) lalu absen
            while (motorX->isMoving() || motorY->isMoving()) {
                xSemaphoreTake(SinyalPosisiTercapai, pdMS_TO_TICKS(20));
                heartbeat_Sequencer = true; // LAPOR MANDOR IWDG!
                if (isAborted) break;       // Tembus seketika jika E-Stop ditekan
            }
            if (isAborted) continue; // Lempar ke Blok A

            motorX->resetPosition();
            motorY->resetPosition();

            // --- TAHAP 2: PULL-OFF (Maju sedikit) ---
            ignoreLimitSwitch = true;
            isNavigating = true;

            motorX->setTargetPulse(Convert_mm_to_PulseX(eepromSettings.homingTravelAfterLimitX));
            motorY->setTargetPulse(Convert_mm_to_PulseY(eepromSettings.homingTravelAfterLimitY));

            while (motorX->isMoving() || motorY->isMoving()) {
                xSemaphoreTake(SinyalPosisiTercapai, pdMS_TO_TICKS(20));
                heartbeat_Sequencer = true; // LAPOR MANDOR IWDG!
                if (isAborted) break;
            }
            if (isAborted) continue;

            ignoreLimitSwitch = false;

            // --- TAHAP 3: TETAPKAN ZERO ABSOLUT ---
            motorX->resetPosition();
            motorY->resetPosition();

            Update_Motor_Param_From_Settings();

            isMachineHomed = true;
            flagMintaHoming = false;
            continue;
        }

        // =========================================================
        // C. SIKLUS LAS OTOMATIS (Pola Zig-Zag)
        // =========================================================
        if (isAutoWeldActive && !isAborted) {

            if (configLas.startKolom > configLas.totalKolom || configLas.startBaris > configLas.totalBaris) {
                indexKolomAktual = 0;
                indexBarisAktual = 0;
            } else {
                indexKolomAktual = configLas.startKolom;
                // [BUG FIXED]: Sebelumnya Anda menulis configLas.totalBaris di sini
                indexBarisAktual = configLas.startBaris;
            }

            if (!isMachineHomed) {
                flagMintaHoming = true;
                continue;
            }

            if (indexBarisAktual >= configLas.totalBaris) {
                isAutoWeldActive = false;
                continue;
            }

            // CEK PAUSE DENGAN WATCHDOG KICK
            while (isPaused && !isAborted) {
                heartbeat_Sequencer = true; // LAPOR MANDOR
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (isAborted) continue;

            Koordinat_t center = CalculateBatteryCenter_mm();

            // =========================================================
            // 4. MINI-LOOP: EKSEKUSI BEBERAPA TITIK DI 1 BATERAI
            // =========================================================
            for (uint16_t i = 0; i < eepromSettings.weldCount; i++) {

                float offsetY_mm = 0.0f;
                if (eepromSettings.weldCount > 1) {
                    float spacing = (float)eepromSettings.spotSpacing_x10 / 10.0f;
                    float totalHeight = spacing * (eepromSettings.weldCount - 1);
                    float startY = -(totalHeight / 2.0f);
                    offsetY_mm = startY + (i * spacing);
                }

                long targetPulseX = Convert_mm_to_PulseX(center.x);
                long targetPulseY = Convert_mm_to_PulseY(center.y + offsetY_mm);

                isNavigating = true;
                motorX->setTargetPulse(targetPulseX);
                motorY->setTargetPulse(targetPulseY);

                while (motorX->isMoving() || motorY->isMoving()) {
                    xSemaphoreTake(SinyalPosisiTercapai, pdMS_TO_TICKS(20));
                    heartbeat_Sequencer = true; // LAPOR MANDOR
                    if (isAborted) break;
                }
                isNavigating = false;
                if (isAborted) break;

                // C. Proses Pengelasan
                HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_SET);

                // MENGHINDARI DELAY PANJANG: Potong jeda Drop Delay menjadi kepingan 10ms
                uint16_t dropDelay = Holding_Registers[4];
                for(uint16_t t = 0; t < dropDelay; t += 10) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    heartbeat_Sequencer = true;
                    if(isAborted) break;
                }
                if (isAborted) break;

                // Tembakan Las (Hanya dikontrol oleh STM32 Timer jika tidak pakai NE555)
                HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(Holding_Registers[5])); // Weld Time (Biasa < 50ms, aman dari IWDG)
                HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);

                HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET);

                // MENGHINDARI DELAY 200MS PANJANG (Waktu Tunggu Pneumatik Naik)
                for(uint16_t t = 0; t < 200; t += 20) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    heartbeat_Sequencer = true;
                    if(isAborted) break;
                }
            }

            if (isAborted) continue;

            // UPDATE INDEX BATERAI
            indexKolomAktual++;
            if (indexKolomAktual >= configLas.totalKolom) {
                indexKolomAktual = 0;
                indexBarisAktual++;
            }

        } else if (flagManualJog && !isAborted) {
            // =========================================================
            // D. MODE MANUAL JOG
            // =========================================================
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

            while (motorX->isMoving() || motorY->isMoving() || motorZ->isMoving()) {
                xSemaphoreTake(SinyalPosisiTercapai, pdMS_TO_TICKS(20));
                heartbeat_Sequencer = true; // LAPOR MANDOR!
                if (isAborted) break;
            }
            isNavigating = false;

            if (isAborted) {
                flagManualJog = false;
                continue;
            }

            flagManualJog = false;
            Holding_Registers[6] = 0; Holding_Registers[7] = 0;
            Holding_Registers[8] = 0; Holding_Registers[9] = 0;
            Holding_Registers[10] = 0; Holding_Registers[11] = 0;

        } else {
            // =========================================================
            // E. MODE STANDBY PURE
            // =========================================================
            vTaskDelay(pdMS_TO_TICKS(50));
            heartbeat_Sequencer = true; // ABSENSI WAJIB SAAT NGANGGUR!
        }
    }
}

// --- CALLBACK INTERUPSI PERANGKAT KERAS (ZERO-BLOCKING) ---
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    // 1. TANGANI KONDISI DARURAT (ABORT)
    if (isAborted) {
        if (motorX) motorX->stop(); // Fungsi stop() otomatis mematikan Hardware Timer
        if (motorY) motorY->stop();
        // Catatan: Semaphore akan otomatis dilepas di blok evaluasi di bawah
    }

    // 2. EKSEKUSI PULSA MOTOR X (Hanya jika tidak di-Pause)
    if (htim->Instance == TIM2) {
        if (motorX && !isPaused) motorX->isr_update();
    }

    // 3. EKSEKUSI PULSA MOTOR Y (Hanya jika tidak di-Pause)
    if (htim->Instance == TIM3) {
        if (motorY && !isPaused) motorY->isr_update();
    }

    // 4. EKSEKUSI PULSA MOTOR Z (Hanya jika tidak di-Pause)
    if (htim->Instance == TIM4) {
        if (motorZ && !isPaused) motorZ->isr_update();
    }

    // 4. SINKRONISASI KE RTOS (Evaluasi Target atau Abort)
    // Jika sedang navigasi dan kedua motor telah berhenti (entah karena sampai atau di-Abort)
    if (isNavigating && !motorX->isMoving() && !motorY->isMoving() && !motorZ->isMoving()) {
        isNavigating = false;

        // Bangunkan Task_Sequencer
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(SinyalPosisiTercapai, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// --- CALLBACK INTERUPSI PIN (LIMIT SWITCH & E-STOP) ---
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	// Abaikan interupsi jika mesin sedang dalam fase mundur (Pull-Off)
	if (ignoreLimitSwitch) return;

	bool isHit = false;

	// 1. Deteksi Limit Switch Sumbu X
	if (GPIO_Pin == LIMIT_X_Pin) {
		if (motorX) motorX->stop(); // Timer X mati instan
		isHit = true;
	}

	// 2. Deteksi Limit Switch Sumbu Y
	if (GPIO_Pin == LIMIT_Y_Pin) {
		if (motorY) motorY->stop(); // Timer Y mati instan
		isHit = true;
	}

	if (GPIO_Pin == LIMIT_Z_Pin) {
		if (motorZ) motorZ->stop(); // Timer Z mati instan
		isHit = true;
	}

	// 3. SINKRONISASI ZERO-BLOCKING RTOS (PENYELAMAT DEADLOCK)
	// Jika mesin sedang bergerak (Homing) dan menabrak switch
	if (isNavigating && isHit) {

		// PASTIKAN KEDUA MOTOR SUDAH BERHENTI
		// Jika mesin homing diagonal, X mungkin nabrak duluan sementara Y masih jalan.
		// Kita baru boleh membangunkan RTOS jika keduanya sudah benar-benar stop.
		if (!motorX->isMoving() && !motorY->isMoving()) {

			isNavigating = false; // Matikan bendera

			// Berikan Semaphore untuk membangunkan Task_Sequencer!
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(SinyalPosisiTercapai, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}




