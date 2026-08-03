/*
 * UtilitasDanPerhitungan.h
 *
 *  Created on: Jul 31, 2026
 *      Author: Tegar
 */

#ifndef SRC_UTILITASDANPERHITUNGAN_H_
#define SRC_UTILITASDANPERHITUNGAN_H_

#include "GlobalData.h"

// =========================================================
// [FUNGSI OTAK CAM: PUSAT BATERAI (ZIG-ZAG)]
// =========================================================
Koordinat_t CalculateBatteryCenter_mm(void);

/**
 * Mengonversi jarak milimeter (Sumbu X) menjadi jumlah pulsa motor
 * @param target_mm Jarak yang diinginkan dalam milimeter (bisa bernilai negatif)
 * @return Jumlah pulsa absolut (32-bit signed integer)
 */
long Convert_mm_to_PulseX(float target_mm);

/**
 * Mengonversi jarak milimeter (Sumbu Y) menjadi jumlah pulsa motor
 * @param target_mm Jarak yang diinginkan dalam milimeter (bisa bernilai negatif)
 * @return Jumlah pulsa absolut (32-bit signed integer)
 */
long Convert_mm_to_PulseY(float target_mm);

/**
 * Mengonversi jarak milimeter (Sumbu Y) menjadi jumlah pulsa motor
 * @param target_mm Jarak yang diinginkan dalam milimeter (bisa bernilai negatif)
 * @return Jumlah pulsa absolut (32-bit signed integer)
 */
long Convert_mm_to_PulseZ(float target_mm);

void Update_Motor_Param_From_Settings(void);
void Update_Motor_Param_Homing_From_Settings(void);

#endif /* SRC_UTILITASDANPERHITUNGAN_H_ */
