/*
 * StepperMotor.cpp
 *
 *  Created on: Jul 13, 2026
 *      Author: Tegar
 */

#include "StepperMotor.h"

// Inisialisasi awal saat objek motor dibuat
StepperMotor::StepperMotor(GPIO_TypeDef* step_port, uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin) {
    stepPort = step_port;
    stepPin = step_pin;
    dirPort = dir_port;
    dirPin = dir_pin;

    currentPosition = 0;
    targetPosition = 0;
    isMoving = false;
}

// Menetapkan target baru dan mengubah status isMoving menjadi true jika target berbeda
void StepperMotor::setTarget(long target) {
    targetPosition = target;
    isMoving = (currentPosition != targetPosition);
}

// Fungsi Pacing: Inilah yang membuat motor melangkah 1 step setiap kali dipanggil
void StepperMotor::update() {
    // Jika sudah sampai target, jangan lakukan apa-apa
    if (!isMoving) return;

    // 1. Tentukan Arah (Direction)
    if (currentPosition < targetPosition) {
        HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_SET);   // Bergerak maju (Positif)
        currentPosition++;
    } else if (currentPosition > targetPosition) {
        HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_RESET); // Bergerak mundur (Negatif)
        currentPosition--;
    }

    // 2. Tembakkan Pulsa STEP (Naik lalu Turun)
    HAL_GPIO_WritePin(stepPort, stepPin, GPIO_PIN_SET);

    /* * SAFETY DELAY UNTUK DRV8825:
     * DRV8825 membutuhkan pulsa HIGH minimal selama 1.9 mikrodetik.
     * Karena STM32F401 berjalan sangat cepat (84 MHz), kita sisipkan
     * sedikit instruksi kosong agar pulsa tidak terlalu sempit dan gagal terbaca.
     */
    for(volatile int i = 0; i < 50; i++) {}

    HAL_GPIO_WritePin(stepPort, stepPin, GPIO_PIN_RESET);

    // 3. Evaluasi apakah target sudah tercapai di langkah ini
    if (currentPosition == targetPosition) {
        isMoving = false;
    }
}

// Menghentikan motor secara paksa (berguna saat menabrak limit switch)
void StepperMotor::stop() {
    targetPosition = currentPosition;
    isMoving = false;
}

// Menjadikan posisi saat ini sebagai Titik Nol (0,0) - dipakai setelah Auto Home
void StepperMotor::resetPosition() {
    currentPosition = 0;
    targetPosition = 0;
    isMoving = false;
}

// Mengambil data koordinat aktual
long StepperMotor::getCurrentPosition() {
    return currentPosition;
}


