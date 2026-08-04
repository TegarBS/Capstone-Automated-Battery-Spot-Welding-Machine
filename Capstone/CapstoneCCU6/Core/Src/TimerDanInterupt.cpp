/*
 * TimerDanInterupt.cpp
 *
 *  Created on: Aug 4, 2026
 *      Author: Tegar
 */

#include "main.h"
#include "GlobalData.h"


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


