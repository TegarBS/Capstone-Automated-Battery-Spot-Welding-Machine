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
Koordinat_t Calculate_Battery_Center_mm(void) {
    Koordinat_t center;

    float start_offset_X = eepromSettings.X_distance_from_0 / 10.0f; // Jarak aman X dari limit switch (mm)
    float start_offset_Y = eepromSettings.Y_distance_from_0 / 10.0f; // Jarak aman Y dari limit switch (mm)

    // 1. Hitung Posisi Y (Selalu naik sesuai baris)
    center.y = (indexBarisAktual * configLas.pitchY) + start_offset_Y;

    // 2. Hitung Posisi X MUTLAK (Absolut Physical)
	// Tidak perlu lagi if-else ganjil genap di sini!
	center.x = (indexKolomAktual * configLas.pitchX) + start_offset_X;

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

/**
 * Mengonversi jarak milimeter (Sumbu Y) menjadi jumlah pulsa motor
 * @param target_mm Jarak yang diinginkan dalam milimeter (bisa bernilai negatif)
 * @return Jumlah pulsa absolut (32-bit signed integer)
 */
long Convert_mm_to_PulseZ(float target_mm) {
    if (target_mm >= 0) {
        return (long)(target_mm * eepromSettings.stepsPerMmZ + 0.5f);
    } else {
        return (long)(target_mm * eepromSettings.stepsPerMmZ - 0.5f);
    }
}

// =========================================================
// [FUNGSI UTILITAS: UPDATE PROFIL KECEPATAN DARI EEPROM]
// Dipanggil 1x setelah booting, atau setiap kali HMI menekan "SAVE"
// =========================================================
void Update_Motor_Param_From_Settings(void) {
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

    // ---------------------------------------------------------
    // 3. UPDATE PROFIL MOTOR Z (Kalkulasi Hz independen)
    // ---------------------------------------------------------
    float calc_maxZ_Hz   = (float)eepromSettings.maxSpeedZ * eepromSettings.stepsPerMmZ / 60.0f;
    float calc_minZ_Hz   = (float)eepromSettings.minSpeedZ * eepromSettings.stepsPerMmZ / 60.0f;
    float calc_accelZ_Hz = (float)eepromSettings.accelZ * eepromSettings.stepsPerMmZ;

    motorZ->setMaxSpeedHz(calc_maxZ_Hz);
    motorZ->setMinSpeedHz(calc_minZ_Hz);
    motorZ->setAccelHz(calc_accelZ_Hz);

    motorX->setInvertDirection(eepromSettings.directionX != 0);
	motorY->setInvertDirection(eepromSettings.directionY != 0);
	motorZ->setInvertDirection(eepromSettings.directionZ != 0);

	// Konversi milimeter ke Pulse, lalu suntikkan ke Motor
	long batasPulseX = Convert_mm_to_PulseX(eepromSettings.maxTravelX);
	long batasPulseY = Convert_mm_to_PulseY(eepromSettings.maxTravelY);
	long batasPulseZ = Convert_mm_to_PulseZ(eepromSettings.maxTravelZ);

	motorX->setMaxPositionLimitPulse(batasPulseX);
	motorY->setMaxPositionLimitPulse(batasPulseY);
	motorZ->setMaxPositionLimitPulse(batasPulseZ);
}

//Motor Z Tidak Homing
void Update_Motor_Param_Homing_From_Settings(void) {
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
}

// ---------------------------------------------------------
// HELPER 2: Jeda Waktu (Delay) Aman dari Watchdog & E-Stop
// Memecah delay panjang menjadi kepingan 10ms
// ---------------------------------------------------------
bool Delay_With_Watchdog(uint32_t total_delay_ms) {
    for(uint32_t t = 0; t < total_delay_ms; t += 10) {
        vTaskDelay(pdMS_TO_TICKS(10));
        heartbeat_Sequencer = true;
        if (isAborted) return false;
    }
    return true;
}
