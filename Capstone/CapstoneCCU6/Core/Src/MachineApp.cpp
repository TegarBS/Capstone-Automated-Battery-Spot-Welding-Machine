#include "MachineApp.h"
#include "StepperMotor.h"
#include "main.h"
#include "Communication.h" // Header Modbus & Queue

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Definisi Pin Fiktif untuk Relay dan Las
#ifndef RELAY_Z_PORT
#define RELAY_Z_PORT GPIOB
#define RELAY_Z_PIN GPIO_PIN_10
#define WELD_PORT GPIOB
#define WELD_PIN GPIO_PIN_11
#endif

// =========================================================
// VARIABEL GLOBAL & RTOS HANDLE
// =========================================================

SemaphoreHandle_t SinyalPosisiTercapai;
volatile bool isNavigating = false;

volatile bool isPaused = false;
volatile bool isAborted = false;

volatile bool ignoreLimitSwitch = false;

// Instansiasi Handle RTOS (di-extern di Communication.h)
QueueHandle_t Queue_Koordinat;
SemaphoreHandle_t Sem_DataDiterima;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern uint8_t rxChar; // Diperlukan untuk pancingan interupsi UART pertama

StepperMotor* motorX;
StepperMotor* motorY;

void Task_Sequencer(void *pvParameters);
void Task_Monitor(void *pvParameters);

// --- FUNGSI INISIALISASI MESIN ---
void MachineApp_Init(void) {
    motorX = new StepperMotor(STEP_X_GPIO_Port, STEP_X_Pin, DIR_X_GPIO_Port, DIR_X_Pin, &htim2);
    motorY = new StepperMotor(STEP_Y_GPIO_Port, STEP_Y_Pin, DIR_Y_GPIO_Port, DIR_Y_Pin, &htim3);

    //Motor X terbalik
    motorX.setInvertDirection(true);

    // Buat Semaphore & Queue
    SinyalPosisiTercapai = xSemaphoreCreateBinary();
    Sem_DataDiterima = xSemaphoreCreateBinary();
    Queue_Koordinat = xQueueCreate(10, sizeof(Koordinat_t));

    // Buat Task RTOS
    xTaskCreate(Task_Sequencer, "Task_Seq", 512, NULL, 2, NULL);

    // Pastikan Task_Komunikasi dipanggil dari Communication.cpp
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

    	// 2. CEK PAUSE
    	// Jika sedang pause, tidurkan Task ini tanpa memblokir sistem lain
    	while (isPaused && !isAborted) {
    		vTaskDelay(pdMS_TO_TICKS(50));
    	}

        // Ambil koordinat dari Queue (Tunggu maksimal 50ms per siklus)
        if (xQueueReceive(Queue_Koordinat, &target, pdMS_TO_TICKS(50)) == pdPASS) {
        	// =========================================================
			// DETEKSI PERINTAH HOMING (KODE -999999)
			// =========================================================
        	if (target.x == -999999 && target.y == -999999) {

				// --- TAHAP 1: SEEK (Cari Limit Switch) ---
				isNavigating = true;
				motorX.setTarget(-999999);
				motorY.setTarget(-999999);

				// Tunggu sampai motor menabrak limit switch & berhenti oleh EXTI
				xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);

				// Reset koordinat sementara agar kita bisa memberikan perintah maju
				motorX.resetPosition();
				motorY.resetPosition();


				// --- TAHAP 2: PULL-OFF (Mundur untuk Melepas Sakelar) ---
				// Matikan sementara respon limit switch agar aman dari getaran sakelar
				ignoreLimitSwitch = true;

				isNavigating = true;
				// Mundur sejauh 200 step (Arah Positif)
				// Catatan: Jika 200 step belum cukup untuk melepas bunyi 'klik' limit switch, naikkan angkanya (misal 500)
				motorX.setTarget(3200);
				motorY.setTarget(3200);

				// Tunggu sampai motor selesai mundur
				xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);

				// Aktifkan kembali perlindungan Limit Switch
				ignoreLimitSwitch = false;


				// --- TAHAP 3: TETAPKAN ZERO ABSOLUT ---
				// Titik setelah mundur ini adalah titik (0,0) yang asli dan aman!
				motorX.resetPosition();
				motorY.resetPosition();

				continue; // Selesai Homing, lanjut ke siklus antrean berikutnya
			}

            // 1. SET TARGET MOTOR (Bergerak ke Titik Las)
            isNavigating = true;
            motorX.setTarget(target.x);
            motorY.setTarget(target.y);

            // 2. TUNGGU MOTOR SAMPAI DI TUJUAN
            xSemaphoreTake(SinyalPosisiTercapai, portMAX_DELAY);

            // Jika dihentikan saat motor sedang bergerak, loncati proses las
			if (isAborted) continue;

			// Cek lagi apakah operator menekan PAUSE persis saat motor sampai
			while (isPaused && !isAborted) vTaskDelay(pdMS_TO_TICKS(50));
			if (isAborted) continue;

            // 3. PROSES PENGELASAN DENGAN PARAMETER DINAMIS MODBUS

            // A. Turunkan Silinder Pneumatik Z
            HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_SET);

            // Baca waktu Drop Delay dari Holding Register 40005 (Beri default 200ms jika kosong)
            uint16_t drop_delay = (Holding_Registers[4] > 0) ? Holding_Registers[4] : 200;
            vTaskDelay(pdMS_TO_TICKS(drop_delay));

            // B. Tembak Trafo Las
            HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_SET);

            // Baca Durasi Las dari Holding Register 40006 (Beri default 30ms jika kosong)
            uint16_t weld_time = (Holding_Registers[5] > 0) ? Holding_Registers[5] : 30;
            vTaskDelay(pdMS_TO_TICKS(weld_time));

            HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);

            // C. Angkat Kembali Silinder Z
            HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(200));

            // Catatan: Tidak perlu lagi HAL_UART_Transmit ACK di sini!
            // Modbus sudah membalas saat paket diterima di Communication.cpp

        } else {
            // --- AREA BEBAS BLOKING (Bisa dipakai untuk membaca tombol Manual Jogging) ---
        }
    }
}

// --- CALLBACK INTERUPSI PERANGKAT KERAS (ZERO-BLOCKING) ---
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    // 1. TANGANI KONDISI DARURAT (ABORT)
    if (isAborted) {
        if (motorX) motorX.stop(); // Fungsi stop() otomatis mematikan Hardware Timer
        if (motorY) motorY.stop();
        // Catatan: Semaphore akan otomatis dilepas di blok evaluasi di bawah
    }

    // 2. EKSEKUSI PULSA MOTOR X (Hanya jika tidak di-Pause)
    if (htim->Instance == TIM2) {
        if (motorX && !isPaused) motorX.isr_update();
    }

    // 3. EKSEKUSI PULSA MOTOR Y (Hanya jika tidak di-Pause)
    if (htim->Instance == TIM3) {
        if (motorY && !isPaused) motorY.isr_update();
    }

    // 4. SINKRONISASI KE RTOS (Evaluasi Target atau Abort)
    // Jika sedang navigasi dan kedua motor telah berhenti (entah karena sampai atau di-Abort)
    if (isNavigating && !motorX.isMoving && !motorY.isMoving) {
        isNavigating = false;

        // Bangunkan Task_Sequencer
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(SinyalPosisiTercapai, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// --- CALLBACK INTERUPSI PIN (LIMIT SWITCH & E-STOP) ---
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

    // (Opsional) Deteksi Emergency Stop (Tidak boleh mengabaikan limit switch)
    // if (GPIO_Pin == ESTOP_Pin) {
    //     motorX.stop(); motorY.stop();
    //     // Matikan relay las / pneumatik
    // }

    // Abaikan interupsi jika mesin sedang dalam fase mundur (Pull-Off)
    if (ignoreLimitSwitch) return;

    // 1. Deteksi Limit Switch Sumbu X
    if (GPIO_Pin == LIMIT_X_Pin) {
        if (motorX) motorX.stop();
    }

    // 2. Deteksi Limit Switch Sumbu Y
    if (GPIO_Pin == LIMIT_Y_Pin) {
        if (motorY) motorY.stop();
    }


}

void Task_Monitor(void *pvParameters) {
    uint32_t counter_led = 0;

    while(1) {
        // --- 1. UPDATE REGISTER MODBUS ---
        if (motorX && motorY) {
            long currentX = motorX.getCurrentPosition();
            long currentY = motorY.getCurrentPosition();

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
