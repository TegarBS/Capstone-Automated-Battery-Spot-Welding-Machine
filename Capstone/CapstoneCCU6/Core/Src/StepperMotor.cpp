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

    currentPosition = 0;
    targetPosition = 0;
    isMoving = false;
    invertDir = false;
    state = IDLE_STATE;

    // Konfigurasi Profil Akselerasi (Clock 4.000.000 Hz)
    minARR = 192;    // Target Kecepatan Puncak: ~20.833 Hz
    startARR = 2000; // Kecepatan Awal (Loncat zona getaran): ~2.000 Hz
}

void StepperMotor::setTarget(long target) {
    if (currentPosition == target) return;

    targetPosition = target;
    stepsToGo = abs(targetPosition - currentPosition);

    // 1. Tentukan Arah
    if (targetPosition > currentPosition) {
        HAL_GPIO_WritePin(dirPort, dirPin, invertDir ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dirPort, dirPin, invertDir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    // 2. Kalkulasi Geometri Trapesium
    accelSteps = 1736; // Jarak akselerasi ideal untuk mencapai 20kHz

    // Jika jarak terlalu pendek, jadikan profil Segitiga
    if (stepsToGo < (accelSteps * 2)) {
        accelSteps = stepsToGo / 2;
    }

    if (accelSteps == 0) accelSteps = 1; // Safety pembagian 0

    // Hitung kemiringan perubahan (Delta) ARR
    arrDelta = (startARR - minARR) / accelSteps;
    if (arrDelta == 0) arrDelta = 1; // Minimal berubah 1 angka

    // 3. Mulai Pergerakan!
    currentARR = startARR;
    state = ACCELERATING;
    isMoving = true;

    // Suntikkan nilai awal ke Hardware Timer lalu NYALAKAN Interupsi
    __HAL_TIM_SET_AUTORELOAD(htim, currentARR);
    HAL_TIM_Base_Start_IT(htim);
}

void StepperMotor::setInvertDirection(bool invert) {
    invertDir = invert;
}

// FUNGSI INI BERJALAN 100% DI DALAM INTERUPSI HARDWARE
void StepperMotor::isr_update() {
    if (!isMoving) return;

    // 1. Tembakkan Pulsa STEP (Hardware Level)
    HAL_GPIO_WritePin(stepPort, stepPin, GPIO_PIN_SET);
    for(volatile int i = 0; i < 15; i++) {} // Safety Delay sangat singkat untuk DRV8825
    HAL_GPIO_WritePin(stepPort, stepPin, GPIO_PIN_RESET);

    // 2. Update Koordinat
    if (targetPosition > currentPosition) currentPosition++;
    else currentPosition--;
    stepsToGo--;

    // 3. MANIPULASI REGISTER FREKUENSI (ARR) DINAMIS
    if (stepsToGo == 0) {
        // TARGET TERCAPAI: Matikan detak Timer agar CPU istirahat total!
        HAL_TIM_Base_Stop_IT(htim);
        isMoving = false;
        state = IDLE_STATE;
    }
    else if (stepsToGo <= accelSteps) {
        // FASE DESELERASI: Perlambat frekuensi (Besarkan nilai ARR)
        state = DECELERATING;
        currentARR += arrDelta;
        if (currentARR > startARR) currentARR = startARR;
        __HAL_TIM_SET_AUTORELOAD(htim, currentARR);
    }
    else if (state == ACCELERATING) {
        // FASE AKSELERASI: Percepat frekuensi (Kecilkan nilai ARR)
        currentARR -= arrDelta;
        if (currentARR <= minARR) {
            currentARR = minARR;
            state = CRUISING;
        }
        __HAL_TIM_SET_AUTORELOAD(htim, currentARR);
    }
}

void StepperMotor::stop() {
    HAL_TIM_Base_Stop_IT(htim); // Matikan Timer instan!
    targetPosition = currentPosition;
    isMoving = false;
    state = IDLE_STATE;
}

void StepperMotor::resetPosition() {
    currentPosition = 0;
    targetPosition = 0;
    isMoving = false;
    state = IDLE_STATE;
}

long StepperMotor::getCurrentPosition() {
    return currentPosition;
}
