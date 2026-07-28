/*
 * MachineApp_Dev.cpp
 *
 *  Created on: Jul 25, 2026
 *      Author: Tegar
 */

#include "MachineApp.h"
#include "StepperMotor.h"
#include "main.h"
#include "Communication.h" // Header Modbus & Queue
#include "GlobalData.h"

// Definisi Pin Fiktif untuk Relay dan Las
#ifndef RELAY_Z_PORT
#define RELAY_Z_PORT GPIOB
#define RELAY_Z_PIN GPIO_PIN_10
#define WELD_PORT GPIOB
#define WELD_PIN GPIO_PIN_11
#endif

void Task_Sequencer(void *pvParameters);
void Task_Monitor(void *pvParameters);
Koordinat_t CalculateNextTarget();

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

// --- TASK RTOS: Eksekutor Sekuensial Las ---
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
            isNavigating = true;

            // --- TAHAP 1: SEEK (Cari Limit Switch) ---
            if (HAL_GPIO_ReadPin(LIMIT_X_GPIO_Port, LIMIT_X_Pin) == LIMIT_X_Pressed) {
                motorX->stop();
            } else {
                motorX->setTarget(-999999);
            }

            if (HAL_GPIO_ReadPin(LIMIT_Y_GPIO_Port, LIMIT_Y_Pin) == LIMIT_Y_Pressed) {
                motorY->stop();
            } else {
                motorY->setTarget(-999999);
            }

            if (motorX->isMoving || motorY->isMoving) {
                xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);
            }

            motorX->resetPosition();
            motorY->resetPosition();

            // --- TAHAP 2: PULL-OFF (Maju agar sakelar terlepas) ---
            ignoreLimitSwitch = true;
            isNavigating = true;
            motorX->setTarget(3200);
            motorY->setTarget(3200);

            xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);
            ignoreLimitSwitch = false;

            // --- TAHAP 3: TETAPKAN ZERO ABSOLUT ---
            motorX->resetPosition();
            motorY->resetPosition();

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

            // 3. KALKULASI TARGET BERIKUTNYA
            Koordinat_t target = CalculateNextTarget();

            // 4. BERGERAK KE TARGET (Zero-Distance Deadlock Fix)
            isNavigating = true;
            motorX->setTarget(target.x);
            motorY->setTarget(target.y);

            if (motorX->isMoving || motorY->isMoving) {
                xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);
            } else {
                isNavigating = false;
            }

            if (isAborted) continue;
            while (isPaused && !isAborted) vTaskDelay(pdMS_TO_TICKS(50));
            if (isAborted) continue;

            // 5. PROSES PENGELASAN
            HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(200));

            HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(30));
            HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(200));

            // 6. UPDATE INDEX BATERAI (Otomatis geser untuk loop selanjutnya)
            indexKolomAktual++;

            if (indexKolomAktual >= configLas.totalKolom) {
                indexKolomAktual = 0;       // Reset kolom ke 0
                indexBarisAktual++;         // Pindah ke baris atasnya
            }

            vTaskDelay(pdMS_TO_TICKS(10)); // Jeda CPU sebelum ke titik berikutnya

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

			flagManualWeld = false; // Kembalikan ke mode standby

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


void Task_Monitor(void *pvParameters) {
    uint32_t counter_led = 0;

    while(1) {
        // --- 1. UPDATE REGISTER MODBUS ---
        if (motorX && motorY) {
            long currentX = motorX->getCurrentPosition();
            long currentY = motorY->getCurrentPosition();

            // Pecah 32-bit (long) menjadi 2 bagian 16-bit (Register)
            Input_Registers[0] = currentX & 0xFFFF;
            Input_Registers[1] = (currentX >> 16) & 0xFFFF;
            Input_Registers[2] = currentY & 0xFFFF;
            Input_Registers[3] = (currentY >> 16) & 0xFFFF;
        }

        // --- 2. KEDIP LED INDIKATOR RTOS ---
        counter_led++;
        // Task berjalan tiap 10ms. 10ms x 100 = 1000ms (1 Detik)
        if (counter_led >= 100) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            counter_led = 0;
        }

        // Tidurkan task selama 10 milidetik
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =========================================================
// [FUNGSI OTAK CAM: KALKULATOR TARGET ZIG-ZAG ABSOLUT]
// =========================================================
Koordinat_t CalculateNextTarget() {
    Koordinat_t target;
    float targetX_mm = 0.0;
    float targetY_mm = 0.0;

    // 1. Hitung Posisi Y (Selalu naik sesuai baris)
    targetY_mm = indexBarisAktual * configLas.pitchY;

    // 2. Hitung Posisi X dengan Pola ZIG-ZAG
    if (indexBarisAktual % 2 == 0) {
        // BARIS GENAP (0, 2, 4): Las bergerak Maju (Kiri ke Kanan)
        targetX_mm = indexKolomAktual * configLas.pitchX;
    } else {
        // BARIS GANJIL (1, 3, 5): Las bergerak Mundur (Kanan ke Kiri)
        targetX_mm = (configLas.totalKolom - 1 - indexKolomAktual) * configLas.pitchX;
    }

    // 3. Konversi Milimeter ke Step Absolut
    target.x = (long)(targetX_mm * STEPS_PER_MM);
    target.y = (long)(targetY_mm * STEPS_PER_MM);

    return target;
}

