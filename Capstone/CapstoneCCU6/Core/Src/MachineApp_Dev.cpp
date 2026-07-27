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

//extern uint8_t rxChar; // Diperlukan untuk pancingan interupsi UART pertama

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
    Queue_Koordinat = xQueueCreate(10, sizeof(Koordinat_t));

    // Buat Task RTOS
    xTaskCreate(Task_Sequencer, "Task_Seq", 512, NULL, 2, NULL);
    xTaskCreate(Task_Komunikasi, "Task_Komunikasi", 512, NULL, 2, NULL);
    xTaskCreate(Task_Monitor, "Task_Mon", 256, NULL, 1, NULL);

    // MENGHIDUPKAN JANTUNG MESIN
    HAL_UART_Receive_IT(&huart1, &rxChar, 1);
}

void MachineApp_Loop(void) {}

// --- TASK RTOS: Eksekutor Sekuensial Las ---
void Task_Sequencer(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(500)); // Jeda inisialisasi hardware
    Koordinat_t target;

    while(1) {

        if (isAborted) {
            xQueueReset(Queue_Koordinat); // Buang sisa antrean titik las!
            isAborted = false;            // Reset bendera
            HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET); // Naikkan pneumatik
            HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);       // Matikan trafo
        }

        // CEK PAUSE
        while (isPaused && !isAborted) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // Ambil koordinat dari Queue (Tunggu maksimal 50ms per siklus)
        if (xQueueReceive(Queue_Koordinat, &target, pdMS_TO_TICKS(50)) == pdPASS) {

            // =========================================================
            // DETEKSI PERINTAH HOMING DARI PC (KODE -999999)
            // =========================================================
            if (target.x == -999999 && target.y == -999999) {

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

                continue; // Selesai Homing, lanjut baca antrean Queue berikutnya
            }

            // =========================================================
            // SIKLUS NORMAL: BERGERAK KE TITIK LAS
            // =========================================================
            isNavigating = true;
            motorX->setTarget(target.x);
            motorY->setTarget(target.y);

            // Tunggu motor sampai di tujuan
            if (motorX->isMoving || motorY->isMoving) {
				// Jika ya, tunggu sampai Timer Hardware selesai dan memberi sinyal Semaphore
				xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);
			} else {
				// Jika target koordinat SAMA dengan posisi saat ini (jarak = 0),
				// motor tidak bergerak dan timer tidak menyala. Langsung matikan bendera navigasi.
				isNavigating = false;
			}

            // Jika dihentikan saat motor sedang bergerak, loncati proses las
            if (isAborted) continue;

            // Cek lagi apakah operator menekan PAUSE persis saat motor sampai
            while (isPaused && !isAborted) vTaskDelay(pdMS_TO_TICKS(50));
            if (isAborted) continue;

            // --- PROSES PENGELASAN ---
            HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(200));

            HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(30));
            HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(200));

        } else {
            // --- AREA BEBAS BLOKING (Manual Jogging dll) ---
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



