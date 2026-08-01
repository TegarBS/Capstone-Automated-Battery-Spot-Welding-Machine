/*
 * StepperMotor.cpp
 *
 * Created on: Jul 13, 2026
 * Author: Tegar
 */

#include "StepperMotor.h"

StepperMotor::StepperMotor(GPIO_TypeDef* step_port, uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin, TIM_HandleTypeDef* timer) {
    stepPort = step_port;
    stepPin = step_pin;
    dirPort = dir_port;
    dirPin = dir_pin;
    htim = timer;

    currentPositionPulse = 0;
    targetPositionPulse = 0;
    isMoving = false;
    invertDir = false;
    state = IDLE_STATE;

    // Inisialisasi Parameter Fisika
	timerClock = 4000000;  // Pastikan sesuai dengan Prescaler di CubeMX

	minSpeedHz = 0;      // Mulai dari 500 Hz (Sangat halus, bebas getar)
	maxSpeedHz = 0;    // Target kecepatan puncak (~20 kHz)

    stepsToGo = 0;
    accelSteps = 0;

    accelHz = 0;
}

void StepperMotor::setTarget(long target) {
    if (currentPositionPulse == target) return;

    targetPositionPulse = target;
    stepsToGo = abs(targetPositionPulse - currentPositionPulse);

    // 1. Tentukan Arah
    if (targetPositionPulse > currentPositionPulse) {
        HAL_GPIO_WritePin(dirPort, dirPin, invertDir ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dirPort, dirPin, invertDir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    // =========================================================
	// 2. KALKULASI JARAK AKSELERASI DINAMIS (Rumus GLBB)
	// Rumus: s = (Vt^2 - V0^2) / 2a
	// =========================================================
	float delta_V_kuadrat = (maxSpeedHz * maxSpeedHz) - (minSpeedHz * minSpeedHz);

	// Cegah pembagian dengan nol jika akselerasi belum diset
	if (accelHz > 0) {
		accelSteps = (uint32_t)(delta_V_kuadrat / (2.0f * accelHz));
	} else {
		accelSteps = 1; // Fallback aman
	}

	// Profil Trapesium: Jika jarak target terlalu dekat, motor tidak akan sempat
	// mencapai kecepatan penuh. Potong jarak akselerasinya menjadi setengah perjalanan.
	if (stepsToGo < (accelSteps * 2)) {
		accelSteps = stepsToGo / 2;
	}
	if (accelSteps == 0) accelSteps = 1;

	// =========================================================
	// 3. HITUNG DELTA FREKUENSI (Kenaikan Hz per step)
	// =========================================================
	deltaSpeedHz = (maxSpeedHz - minSpeedHz) / (float)accelSteps;

    // 4. Mulai Pergerakan dari Frekuensi Terendah
    currentSpeedHz = minSpeedHz;
    state = ACCELERATING;
    isMoving = true;

    // Kalkulasi nilai ARR perdana dan jalankan timer
    uint32_t startARR = (uint32_t)(timerClock / currentSpeedHz);
    __HAL_TIM_SET_AUTORELOAD(htim, startARR);
    HAL_TIM_Base_Start_IT(htim);
}

void StepperMotor::setInvertDirection(bool invert) {
    invertDir = invert;
}


void StepperMotor::setMinSpeedHz(float minSpeedHz){
	this->minSpeedHz = minSpeedHz;
}

// Mengubah kecepatan puncak secara dinamis (on-the-fly)
void StepperMotor::setMaxSpeedHz(float maxSpeedHz){
	this->maxSpeedHz = maxSpeedHz;
}

void StepperMotor::setAccelHz(float accelHz){
	this->accelHz = accelHz;
}

// FUNGSI INI BERJALAN 100% DI DALAM INTERUPSI HARDWARE
void StepperMotor::isr_update(void) {
    if (!isMoving) return;

    // 1. Tembakkan Pulsa STEP
    HAL_GPIO_WritePin(stepPort, stepPin, GPIO_PIN_SET);
    for(volatile int i = 0; i < 100; i++) {} // Delay aman ~5 microsecond
    HAL_GPIO_WritePin(stepPort, stepPin, GPIO_PIN_RESET);

    // 2. Update Koordinat
    if (targetPositionPulse > currentPositionPulse) currentPositionPulse++;
    else currentPositionPulse--;
    stepsToGo--;

    // 3. KALKULASI PROFIL KECEPATAN (BERDASARKAN FREKUENSI)
    if (stepsToGo == 0) {
        // TARGET TERCAPAI
        HAL_TIM_Base_Stop_IT(htim);
        isMoving = false;
        state = IDLE_STATE;

        // (Panggil Semaphore RTOS Anda di file MachineApp.cpp seperti sebelumnya)
        return;
    }
    else if (stepsToGo <= accelSteps) {
        // FASE DESELERASI: Turunkan Frekuensi
        state = DECELERATING;
        currentSpeedHz -= deltaSpeedHz;
        if (currentSpeedHz < minSpeedHz) currentSpeedHz = minSpeedHz; // Safety limit
    }
    else if (state == ACCELERATING) {
        // FASE AKSELERASI: Naikkan Frekuensi
        currentSpeedHz += deltaSpeedHz;
        if (currentSpeedHz >= maxSpeedHz) {
            currentSpeedHz = maxSpeedHz;
            state = CRUISING;
        }
    }

    // 4. Konversi Frekuensi Aktual menjadi nilai ARR untuk Timer
    // Pembagian desimal ini dieksekusi sangat cepat berkat Hardware FPU STM32F4!
    uint32_t nextARR = (uint32_t)(timerClock / currentSpeedHz);
    __HAL_TIM_SET_AUTORELOAD(htim, nextARR);
}

void StepperMotor::stop(void) {
    HAL_TIM_Base_Stop_IT(htim); // Matikan Timer instan!
    targetPositionPulse = currentPositionPulse;
    isMoving = false;
    state = IDLE_STATE;
}

void StepperMotor::resetPosition(void) {
    currentPositionPulse = 0;
    targetPositionPulse = 0;
    isMoving = false;
    state = IDLE_STATE;
}

long StepperMotor::getCurrentPositionPulse(void) {
    return currentPositionPulse;
}
