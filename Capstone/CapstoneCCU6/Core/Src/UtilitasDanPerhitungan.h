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
Koordinat_t Calculate_Battery_Center_mm(void);

/**
 * Mengonversi jarak milimeter (Sumbu X) menjadi jumlah pulsa motor
 * @param target_mm Jarak yang diinginkan dalam milimeter (bisa bernilai negatif)
 * @return Jumlah pulsa absolut (32-bit signed integer)
 */
int32_t Convert_mm_to_PulseX(float target_mm);

/**
 * Mengonversi jarak milimeter (Sumbu Y) menjadi jumlah pulsa motor
 * @param target_mm Jarak yang diinginkan dalam milimeter (bisa bernilai negatif)
 * @return Jumlah pulsa absolut (32-bit signed integer)
 */
int32_t Convert_mm_to_PulseY(float target_mm);

/**
 * Mengonversi jarak milimeter (Sumbu Y) menjadi jumlah pulsa motor
 * @param target_mm Jarak yang diinginkan dalam milimeter (bisa bernilai negatif)
 * @return Jumlah pulsa absolut (32-bit signed integer)
 */
int32_t Convert_mm_to_PulseZ(float target_mm);

void Update_Motor_Param_From_Settings(void);
void Update_Motor_Param_Homing_From_Settings(void);

// ---------------------------------------------------------
// HELPER 2: Jeda Waktu (Delay) Aman dari Watchdog & E-Stop
// Memecah delay panjang menjadi kepingan 10ms
// ---------------------------------------------------------
bool Delay_With_Watchdog(uint32_t total_delay_ms, volatile bool &heartbeat);

void Silinder_Up(void);
void Silinder_Down(void);
void Silinder_Hold(void);

#endif /* SRC_UTILITASDANPERHITUNGAN_H_ */
