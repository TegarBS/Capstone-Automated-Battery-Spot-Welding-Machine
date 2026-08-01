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

// Definisi Pin Fiktif untuk Relay dan Las
#ifndef RELAY_Z_PORT
#define RELAY_Z_PORT GPIOB
#define RELAY_Z_PIN GPIO_PIN_10
#define WELD_PORT GPIOB
#define WELD_PIN GPIO_PIN_11
#endif

void Task_Sequencer(void *pvParameters);
void Task_Monitor(void *pvParameters);


// --- FUNGSI INISIALISASI MESIN ---
void MachineApp_Init(void) {
    motorX = new StepperMotor(STEP_X_GPIO_Port, STEP_X_Pin, DIR_X_GPIO_Port, DIR_X_Pin, &htim2);
    motorY = new StepperMotor(STEP_Y_GPIO_Port, STEP_Y_Pin, DIR_Y_GPIO_Port, DIR_Y_Pin, &htim3);

    //Motor X terbalik
    motorX->setInvertDirection(true);

    // Buat Semaphore & Queue
    SinyalPosisiTercapai = xSemaphoreCreateBinary();
    Sem_DataDiterima = xSemaphoreCreateBinary();

    // Buat Task RTOS
    xTaskCreate(Task_Sequencer, "Task_Seq", 512, NULL, 2, NULL);
    xTaskCreate(Task_Komunikasi, "Task_Komunikasi", 512, NULL, 2, NULL);
    xTaskCreate(Task_Monitor, "Task_Mon", 256, NULL, 1, NULL);

    // MENGHIDUPKAN JANTUNG MESIN
    HAL_UART_Receive_IT(&huart1, &rxChar, 1);
}

void MachineApp_Loop(void) {}

// --- TASK RTOS: Eksekutor Sekuensial Las (OTAK CAM) ---
void Task_Sequencer(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(500)); // Jeda inisialisasi hardware

    while(1) {

        // =========================================================
        // A. KONDISI DARURAT (ABORT)
        // =========================================================
        if (isAborted) {
            isAutoWeldActive = false; // Batalkan siklus otomatis
            isAborted = false;        // Reset bendera
            HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET); // Naikkan pneumatik
            HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);       // Matikan trafo
        }

        // =========================================================
        // B. FITUR HOMING MANDIRI (Pre-Flight Check + Pull Off)
        // =========================================================
        if (flagMintaHoming) {

        	Update_Motor_Speed_Homing_From_Settings();

			// --- TAHAP 1: SEEK (Mencari Limit Switch dengan Pre-Flight Check) ---
			isNavigating = true;

			// Cek Sumbu X: Jika sudah tertekan dari awal, stop. Jika belum, mundur.
			if (HAL_GPIO_ReadPin(LIMIT_X_GPIO_Port, LIMIT_X_Pin) == LIMIT_X_Pressed) {
				motorX->stop();
			} else {
				motorX->setTarget(-999999);
			}

			// Cek Sumbu Y: Jika sudah tertekan dari awal, stop. Jika belum, mundur.
			if (HAL_GPIO_ReadPin(LIMIT_Y_GPIO_Port, LIMIT_Y_Pin) == LIMIT_Y_Pressed) {
				motorY->stop();
			} else {
				motorY->setTarget(-999999);
			}

			// Tunggu Semaphore HANYA JIKA ada motor yang benar-benar bergerak
			if (motorX->isMoving || motorY->isMoving) {
				xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);
			}

			// Reset koordinat sementara
			motorX->resetPosition();
			motorY->resetPosition();


			// --- TAHAP 2: PULL-OFF (Maju sedikit agar sakelar terlepas) ---
			ignoreLimitSwitch = true; // Matikan EXTI sementara agar aman dari getaran (bouncing)
			isNavigating = true;

			// Maju sejauh 3200 step (Sesuaikan angkanya jika dirasa kurang jauh)
			motorX->setTarget(3200);
			motorY->setTarget(3200);

			// Tunggu sampai motor selesai maju
			xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);

			ignoreLimitSwitch = false; // Aktifkan kembali EXTI


			// --- TAHAP 3: TETAPKAN ZERO ABSOLUT ---
			// Titik setelah mundur ini adalah titik (0,0) yang asli dan aman!
			motorX->resetPosition();
			motorY->resetPosition();

			Update_Motor_Speed_From_Settings();

            flagMintaHoming = false; // Matikan flag setelah selesai
            continue; // Ulangi loop
		}

        // =========================================================
        // C. SIKLUS LAS OTOMATIS (Pola Zig-Zag)
        // =========================================================
        if (isAutoWeldActive) {

            // 1. Cek apakah seluruh pekerjaan sudah tuntas?
            if (indexBarisAktual >= configLas.totalBaris) {
                isAutoWeldActive = false; // Matikan mesin
                continue;
            }

            // 2. CEK PAUSE
            while (isPaused && !isAborted) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (isAborted) continue;

			// 3. KALKULASI TITIK PUSAT BATERAI SAAT INI
			Koordinat_t center = CalculateBatteryCenter_mm();

			// =========================================================
			// 4. MINI-LOOP: EKSEKUSI BEBERAPA TITIK DI 1 BATERAI
			// =========================================================
			for (uint16_t i = 0; i < eepromSettings.weldCount; i++) {

				// A. Kalkulasi Pergeseran Simetris di SUMBU Y (Offset Vertikal)
				// Jika jarum las sejajar sumbu X, kita menggeser Y agar membentuk KOTAK
				float offsetY_mm = 0.0f;
				if (eepromSettings.weldCount > 1) {
					float spacing = (float)eepromSettings.spotSpacing_x10 / 10.0f;
					float totalHeight = spacing * (eepromSettings.weldCount - 1);
					// Mulai dari titik bawah, bergerak ke atas (atau sebaliknya)
					float startY = -(totalHeight / 2.0f);
					offsetY_mm = startY + (i * spacing);
				}

				// B. Konversi ke Pulse & Bergerak!
				// Sumbu X tetap di titik tengah baterai, Sumbu Y yang ditambah pergeseran
				long targetPulseX = Convert_mm_to_PulseX(center.x);
				long targetPulseY = Convert_mm_to_PulseY(center.y + offsetY_mm);

				isNavigating = true;
				motorX->setTarget(targetPulseX);
				motorY->setTarget(targetPulseY);

				if (motorX->isMoving || motorY->isMoving) {
					xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);
				} else {
					isNavigating = false;
				}

				// Pengecekan Darurat di tengah-tengah Mini-Loop
				if (isAborted) break;
				while (isPaused && !isAborted) vTaskDelay(pdMS_TO_TICKS(50));
				if (isAborted) break;

				// C. Proses Pengelasan (Menghasilkan 2 titik secara fisik karena 2 jarum X)
				HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_SET);
				vTaskDelay(pdMS_TO_TICKS(Holding_Registers[4])); // Drop Delay

				HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_SET);
				vTaskDelay(pdMS_TO_TICKS(Holding_Registers[5])); // Weld Time
				HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET);
				vTaskDelay(pdMS_TO_TICKS(200)); // Waktu tunggu pneumatik naik sempurna

				// Jeda sepersekian detik sebelum motor bergeser vertikal ke titik sebelahnya
				vTaskDelay(pdMS_TO_TICKS(50));
			}

			// Jika mesin dihentikan saat mini-loop berjalan, lompat siklus
			if (isAborted) continue;

			// 6. UPDATE INDEX BATERAI
			indexKolomAktual++;

			if (indexKolomAktual >= configLas.totalKolom) {
				indexKolomAktual = 0;
				indexBarisAktual++;
			}

			vTaskDelay(pdMS_TO_TICKS(10));

        } else if (flagManualWeld) {
			// =========================================================
			// D. MODE MANUAL (Single Jog & Weld)
			// =========================================================

			// 1. Ambil koordinat 32-bit dari Holding Register 6, 7 (X) dan 8, 9 (Y)
			long targetManX = ((long)Holding_Registers[7] << 16) | Holding_Registers[6];
			long targetManY = ((long)Holding_Registers[9] << 16) | Holding_Registers[8];

			// 2. Bergerak ke Target
			isNavigating = true;
			motorX->setTarget(targetManX);
			motorY->setTarget(targetManY);

			// Cegah deadlock jika jarak = 0
			if (motorX->isMoving || motorY->isMoving) {
				xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);
			} else {
				isNavigating = false;
			}

			if (isAborted) {
				flagManualWeld = false;
				continue;
			}

			// 3. Proses Las Manual
			HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_SET);
			vTaskDelay(pdMS_TO_TICKS(200));

			HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_SET);
			vTaskDelay(pdMS_TO_TICKS(30));
			HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET);
			vTaskDelay(pdMS_TO_TICKS(200));

			flagManualWeld = false; // Kembalikan ke mode standb

		} else {
			// =========================================================
			// E. MODE STANDBY PURE (Mesin nganggur menunggu perintah PC)
			// =========================================================
			vTaskDelay(pdMS_TO_TICKS(50));
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

    // 4. SINKRONISASI KE RTOS (Evaluasi Target atau Abort)
    // Jika sedang navigasi dan kedua motor telah berhenti (entah karena sampai atau di-Abort)
    if (isNavigating && !motorX->isMoving && !motorY->isMoving) {
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

	// 3. SINKRONISASI ZERO-BLOCKING RTOS (PENYELAMAT DEADLOCK)
	// Jika mesin sedang bergerak (Homing) dan menabrak switch
	if (isNavigating && isHit) {

		// PASTIKAN KEDUA MOTOR SUDAH BERHENTI
		// Jika mesin homing diagonal, X mungkin nabrak duluan sementara Y masih jalan.
		// Kita baru boleh membangunkan RTOS jika keduanya sudah benar-benar stop.
		if (!motorX->isMoving && !motorY->isMoving) {

			isNavigating = false; // Matikan bendera

			// Berikan Semaphore untuk membangunkan Task_Sequencer!
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(SinyalPosisiTercapai, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}



/**
 * Task monitoring lokasi real-time
 * Blinking LED untuk monitor kondisi STM32
 *
 */
void Task_Monitor(void *pvParameters) {
    uint32_t counter_led = 0;

    while(1) {
        // --- 1. UPDATE REGISTER MODBUS (MONITORING POSISI) ---
        if (motorX && motorY) {
            long currentX_pulse = motorX->getCurrentPositionPulse();
            long currentY_pulse = motorY->getCurrentPositionPulse();

            // KONVERSI KE MILIMETER DENGAN PRESISI x100
            // Rumus: (Pulse * 100) / Steps_per_mm
            long currentX_mm = (currentX_pulse * 100) / eepromSettings.stepsPerMmX;
            long currentY_mm = (currentY_pulse * 100) / eepromSettings.stepsPerMmY;

            // Pecah 32-bit (long) menjadi 2 bagian 16-bit untuk Input Register Modbus
            Input_Registers[0] = currentX_mm & 0xFFFF;
            Input_Registers[1] = (currentX_mm >> 16) & 0xFFFF;
            Input_Registers[2] = currentY_mm & 0xFFFF;
            Input_Registers[3] = (currentY_mm >> 16) & 0xFFFF;
        }

        // --- 2. KEDIP LED INDIKATOR RTOS ---
        counter_led++;
        // Task berjalan tiap 100ms. 100ms x 10 = 1000ms (1 Detik)
        if (counter_led >= 10) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            counter_led = 0;
        }

        // Tidurkan task selama 100 milidetik
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


