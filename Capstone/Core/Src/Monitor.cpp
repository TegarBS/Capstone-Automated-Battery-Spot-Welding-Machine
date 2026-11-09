/*
 * Monitor.cpp
 *
 *  Created on: Aug 4, 2026
 *      Author: Tegar
 */

#include "Monitor.h"
#include "GlobalData.h"

/**
 * Task monitoring lokasi real-time
 * Blinking LED untuk monitor kondisi STM32
 *
 */

uint32_t lastTick = 0;

void Task_Monitor(void *pvParameters) {

	for(uint32_t t = 0; t < 200; t += 10) {
		vTaskDelay(pdMS_TO_TICKS(10));
		HAL_IWDG_Refresh(&hiwdg);
	}

	uint32_t counter_led = 0;
	lastTick = HAL_GetTick();   // TAMBAHAN 1: titik awal hitung

    while(1) {

    	if (heartbeat_Sequencer == true && heartbeat_Komunikasi == true) {

			// Sistem Sehat Total! Tendang Watchdog Hardware
			HAL_IWDG_Refresh(&hiwdg);

	        // --- 2. KEDIP LED INDIKATOR SISTEM SEHAT ---
	        counter_led++;

	        if (counter_led >= 10) {
	            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	            counter_led = 0;
	        }

			// Turunkan kembali bendera absen untuk ujian di siklus berikutnya
			heartbeat_Sequencer = false;
			heartbeat_Komunikasi = false;

			lastTick = HAL_GetTick();   // TAMBAHAN 2: catat waktu blok ini dieksekusi
			isErrorSave = false;        // (opsional, sesuai komentar Anda: normal kembali)
		}

    	// TAMBAHAN 3: blok di atas belum dieksekusi selama 100 ms
    	if ((HAL_GetTick() - lastTick) >= 100) {
    		isErrorSave = true;
    	}

        // --- 1. UPDATE REGISTER MODBUS (MONITORING POSISI) ---
		int32_t currentX_pulse = motorX.getCurrentPositionPulse();
		int32_t currentY_pulse = motorY.getCurrentPositionPulse();
		int32_t currentZ_pulse = motorZ.getCurrentPositionPulse();

		int32_t currentX_mm = (eepromSettings.stepsPerMmX == 0) ? 0 : ((currentX_pulse * 100) / eepromSettings.stepsPerMmX);
		int32_t currentY_mm = (eepromSettings.stepsPerMmY == 0) ? 0 : ((currentY_pulse * 100) / eepromSettings.stepsPerMmY);
		int32_t currentZ_mm = (eepromSettings.stepsPerMmZ == 0) ? 0 : ((currentZ_pulse * 100) / eepromSettings.stepsPerMmZ);

		// Pecah 32-bit (int32_t) menjadi 2 bagian 16-bit untuk Input Register Modbus
		Input_Registers[0] = currentX_mm & 0xFFFF;
		Input_Registers[1] = (currentX_mm >> 16) & 0xFFFF;
		Input_Registers[2] = currentY_mm & 0xFFFF;
		Input_Registers[3] = (currentY_mm >> 16) & 0xFFFF;
		Input_Registers[4] = currentZ_mm & 0xFFFF;
		Input_Registers[5] = (currentZ_mm >> 16) & 0xFFFF;

		Input_Registers[7] = indexKolomAktual + 1;
		Input_Registers[8] = indexBarisAktual + 1;

		// Alert ke HMI
		Holding_Registers[12] = kode_alarm_sistem;

		Holding_Registers[70] = mesinOdo.totalTitikLas & 0xFFFF;
		Holding_Registers[71] = (mesinOdo.totalTitikLas >> 16) & 0xFFFF;
		Holding_Registers[72] = mesinOdo.totalJarakXmm & 0xFFFF;
		Holding_Registers[73] = (mesinOdo.totalJarakXmm >> 16) & 0xFFFF;
		Holding_Registers[74] = mesinOdo.totalJarakYmm & 0xFFFF;
		Holding_Registers[75] = (mesinOdo.totalJarakYmm >> 16) & 0xFFFF;

        // Tidurkan task selama 50 milidetik
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


