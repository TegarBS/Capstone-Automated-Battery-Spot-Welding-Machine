/*
 * MachineApp.cpp
 *
 *  Created on: Jul 13, 2026
 *      Author: Tegar
 */

// Nanti kita akan meng-include library StepperMotor dan FreeRTOS di sini
#include "MachineApp.h"
#include "StepperMotor.h"
#include "main.h" // Wajib disertakan untuk mengakses definisi HAL dan pin GPIO

// [TAMBAHAN UNTUK LANGKAH 5: Definisi Pin Fiktif untuk Relay dan Las]
// Nanti sesuaikan port dan pin ini dengan konfigurasi aktual di file .ioc Anda
#define RELAY_Z_PORT GPIOB
#define RELAY_Z_PIN GPIO_PIN_10
#define WELD_PORT GPIOB
#define WELD_PIN GPIO_PIN_11

enum MachineState {
    STATE_IDLE,
    STATE_MOVE_XY,
    STATE_DROP_Z,
    STATE_WELD,
    STATE_LIFT_Z
};

// Variabel pelacak status mesin saat ini dan timer non-blocking
MachineState currentState = STATE_IDLE;
uint32_t stateDelayTimer = 0;

// Mengambil variabel timer TIM2 yang sudah dibuat oleh CubeMX di main.c
extern TIM_HandleTypeDef htim2;

// Membuat pointer untuk objek motor X dan Y
StepperMotor* motorX;
StepperMotor* motorY;

// --- FUNGSI INISIALISASI MESIN ---
void MachineApp_Init(void) {
    /* * Inisialisasi Objek Motor
     * Sesuaikan dengan konfigurasi pin yang sudah diatur di CubeMX (Langkah 1)
     * Contoh: STEP_X di PB13, DIR_X di PB12
     */
	motorX = new StepperMotor(STEP_X_GPIO_Port, STEP_X_Pin, DIR_X_GPIO_Port, DIR_X_Pin);

	motorY = new StepperMotor(STEP_Y_GPIO_Port, STEP_Y_Pin, DIR_Y_GPIO_Port, DIR_Y_Pin);


    // [UJI COBA]: Memberikan target fiktif agar motor langsung bergerak saat mesin menyala
    motorX->setTarget(6400); // Bergerak maju 6400 step (1 putaran penuh dengan 1/32 microstepping)
    motorY->setTarget(-3200); // Bergerak mundur setengah putaran

    // [TAMBAHAN UNTUK LANGKAH 5: Pemicu awal FSM]
    // Kita paksa mesin langsung masuk ke mode gerak saat dinyalakan (untuk uji coba)
    currentState = STATE_MOVE_XY;

    /* * MENGHIDUPKAN JANTUNG MESIN
     * Mengaktifkan Timer 2 dengan mode Interrupt.
     * Mulai dari baris ini dieksekusi, fungsi HAL_TIM_PeriodElapsedCallback
     * akan dipanggil secara otomatis setiap 100 mikrodetik!
     */
    HAL_TIM_Base_Start_IT(&htim2);
}

// --- FUNGSI LOOP MESIN ---
void MachineApp_Loop(void) {
    // [TAMBAHAN UNTUK LANGKAH 5: Logika FSM Sekuensial (Super Loop)]
    // CPU akan terus mengevaluasi switch-case ini jutaan kali per detik tanpa hambatan!

    switch (currentState) {
        case STATE_IDLE:
            // Mesin diam, siap menerima target koordinat baterai berikutnya
            break;

        case STATE_MOVE_XY:
            // Cek apakah kedua motor sudah sampai di tujuan (isMoving == false)
            if (!motorX->isMoving(void) && !motorY->isMoving(void)) {
                // Motor sampai! Pindah ke fase berikutnya: Turunkan Pneumatik Z
                currentState = STATE_DROP_Z;

                // Aktifkan Relay Silinder Pneumatik (Turun)
                HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_SET);

                // Catat stempel waktu (timestamp) saat ini
                stateDelayTimer = HAL_GetTick();
            }
            break;

        case STATE_DROP_Z:
            // NON-BLOCKING DELAY: Tunggu 200ms agar silinder mekanis benar-benar turun
            if (HAL_GetTick() - stateDelayTimer >= 200) {
                currentState = STATE_WELD;

                // Tembak trafo las (Kirim sinyal ke Optocoupler)
                HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_SET);

                stateDelayTimer = HAL_GetTick(); // Catat waktu mulai mengelas
            }
            break;

        case STATE_WELD:
            // NON-BLOCKING DELAY: Tahan sinyal las selama 30ms (Weld Time Presisi)
            if (HAL_GetTick() - stateDelayTimer >= 30) {

                // Matikan trafo las secepatnya!
                HAL_GPIO_WritePin(WELD_PORT, WELD_PIN, GPIO_PIN_RESET);

                currentState = STATE_LIFT_Z;

                // Angkat kembali silinder pneumatik
                HAL_GPIO_WritePin(RELAY_Z_PORT, RELAY_Z_PIN, GPIO_PIN_RESET);

                stateDelayTimer = HAL_GetTick(); // Catat waktu mulai mengangkat
            }
            break;

        case STATE_LIFT_Z:
            // NON-BLOCKING DELAY: Tunggu 200ms agar silinder naik sempurna
            if (HAL_GetTick() - stateDelayTimer >= 200) {
                // Siklus 1 titik las SELESAI! Mesin kembali istirahat.
                currentState = STATE_IDLE;
            }
            break;
    }
}

// --- CALLBACK INTERUPSI PERANGKAT KERAS (HARDWARE ISR) ---
/* * Fungsi ini dioverride (ditimpa) dari fungsi bawaan STM32.
 * Hati-hati: Jangan pernah menaruh fungsi delay() atau print Serial yang berat
 * di dalam fungsi ini karena akan menyebabkan sistem crash (HardFault).
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // Pastikan interupsi ini benar-benar berasal dari TIM2 (bukan timer lain)
    if (htim->Instance == TIM2) {

        // Panggil fungsi update() untuk memicu langkah (STEP)
        // Pengecekan pointer (motor != nullptr) mencegah crash jika timer menyala sebelum motor diinisialisasi
        if (motorX) motorX->update();
        if (motorY) motorY->update();

    }
}
