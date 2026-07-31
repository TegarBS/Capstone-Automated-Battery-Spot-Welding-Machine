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

