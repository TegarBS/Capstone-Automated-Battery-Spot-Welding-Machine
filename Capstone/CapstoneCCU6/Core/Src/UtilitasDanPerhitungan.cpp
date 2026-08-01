/*
 * UtilitasDanPerhitungan.cpp
 *
 *  Created on: Jul 31, 2026
 *      Author: Tegar
 */

#include "UtilitasDanPerhitungan.h"

// =========================================================
// [FUNGSI OTAK CAM: PUSAT BATERAI (ZIG-ZAG)]
// =========================================================
Koordinat_t CalculateBatteryCenter_mm() {
    Koordinat_t center;

    // 1. Hitung Posisi Y (Selalu naik sesuai baris)
    center.y = indexBarisAktual * configLas.pitchY;

    // 2. Hitung Posisi X dengan Pola ZIG-ZAG
    if (indexBarisAktual % 2 == 0) {
        center.x = indexKolomAktual * configLas.pitchX;
    } else {
        center.x = (configLas.totalKolom - 1 - indexKolomAktual) * configLas.pitchX;
    }

    return center;
}

// =========================================================
// [FUNGSI UTILITAS: KONVERSI MILIMETER KE PULSE/STEP]
// =========================================================

/**
 * Mengonversi jarak milimeter (Sumbu X) menjadi jumlah pulsa motor
 * @param target_mm Jarak yang diinginkan dalam milimeter (bisa bernilai negatif)
 * @return Jumlah pulsa absolut (32-bit signed integer)
 */
long Convert_mm_to_PulseX(float target_mm) {
    // Trik CNC: Tambahkan 0.5f untuk pembulatan (rounding) ke nearest integer.
    // Jika tidak, angka 10.9 step akan dipotong (truncate) menjadi 10 step oleh C++.
    if (target_mm >= 0) {
        return (long)(target_mm * eepromSettings.stepsPerMmX + 0.5f);
    } else {
        // Untuk pergerakan mundur (negatif)
        return (long)(target_mm * eepromSettings.stepsPerMmX - 0.5f);
    }
}

/**
 * Mengonversi jarak milimeter (Sumbu Y) menjadi jumlah pulsa motor
 * @param target_mm Jarak yang diinginkan dalam milimeter (bisa bernilai negatif)
 * @return Jumlah pulsa absolut (32-bit signed integer)
 */
long Convert_mm_to_PulseY(float target_mm) {
    if (target_mm >= 0) {
        return (long)(target_mm * eepromSettings.stepsPerMmY + 0.5f);
    } else {
        return (long)(target_mm * eepromSettings.stepsPerMmY - 0.5f);
    }
}

// =========================================================
// [FUNGSI UTILITAS: UPDATE PROFIL KECEPATAN DARI EEPROM]
// Dipanggil 1x setelah booting, atau setiap kali HMI menekan "SAVE"
// =========================================================
void Update_Motor_Speed_From_Settings(void) {
    // ---------------------------------------------------------
    // 1. UPDATE PROFIL MOTOR X (Kalkulasi Hz independen)
    // ---------------------------------------------------------
    float calc_maxX_Hz   = (float)eepromSettings.maxSpeedX * eepromSettings.stepsPerMmX / 60.0f;
    float calc_minX_Hz   = (float)eepromSettings.minSpeedX * eepromSettings.stepsPerMmX / 60.0f;
    float calc_accelX_Hz = (float)eepromSettings.accelX * eepromSettings.stepsPerMmX;

    motorX->setMaxSpeedHz(calc_maxX_Hz);
    motorX->setMinSpeedHz(calc_minX_Hz);
    motorX->setAccelHz(calc_accelX_Hz);

    // ---------------------------------------------------------
    // 2. UPDATE PROFIL MOTOR Y (Kalkulasi Hz independen)
    // ---------------------------------------------------------
    float calc_maxY_Hz   = (float)eepromSettings.maxSpeedY * eepromSettings.stepsPerMmY / 60.0f;
    float calc_minY_Hz   = (float)eepromSettings.minSpeedY * eepromSettings.stepsPerMmY / 60.0f;
    float calc_accelY_Hz = (float)eepromSettings.accelY * eepromSettings.stepsPerMmY;

    motorY->setMaxSpeedHz(calc_maxY_Hz);
    motorY->setMinSpeedHz(calc_minY_Hz);
    motorY->setAccelHz(calc_accelY_Hz);

    motorX->setInvertDirection(eepromSettings.directionX != 0);
	motorY->setInvertDirection(eepromSettings.directionY != 0);
	motorZ->setInvertDirection(eepromSettings.directionZ != 0);
}

void Update_Motor_Speed_Homing_From_Settings(void) {
    // ---------------------------------------------------------
    // 1. UPDATE PROFIL MOTOR X (Kalkulasi Hz independen)
    // ---------------------------------------------------------
    float calc_maxX_Hz   = (float)eepromSettings.maxSpeedHomingX * eepromSettings.stepsPerMmX / 60.0f;
    float calc_minX_Hz   = (float)eepromSettings.minSpeedX * eepromSettings.stepsPerMmX / 60.0f;
    float calc_accelX_Hz = (float)eepromSettings.accelX * eepromSettings.stepsPerMmX;

    motorX->setMaxSpeedHz(calc_maxX_Hz);
    motorX->setMinSpeedHz(calc_minX_Hz);
    motorX->setAccelHz(calc_accelX_Hz);

    // ---------------------------------------------------------
    // 2. UPDATE PROFIL MOTOR Y (Kalkulasi Hz independen)
    // ---------------------------------------------------------
    float calc_maxY_Hz   = (float)eepromSettings.maxSpeedHomingY * eepromSettings.stepsPerMmY / 60.0f;
    float calc_minY_Hz   = (float)eepromSettings.minSpeedY * eepromSettings.stepsPerMmY / 60.0f;
    float calc_accelY_Hz = (float)eepromSettings.accelY * eepromSettings.stepsPerMmY;

    motorY->setMaxSpeedHz(calc_maxY_Hz);
    motorY->setMinSpeedHz(calc_minY_Hz);
    motorY->setAccelHz(calc_accelY_Hz);

    motorX->setInvertDirection(eepromSettings.directionX != 0);
	motorY->setInvertDirection(eepromSettings.directionY != 0);
	motorZ->setInvertDirection(eepromSettings.directionZ != 0);
}
