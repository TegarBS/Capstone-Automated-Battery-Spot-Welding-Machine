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
void Task_Monitor(void *pvParameters) {

	for(uint32_t t = 0; t < 1000; t += 10) {
		vTaskDelay(pdMS_TO_TICKS(10));
		HAL_IWDG_Refresh(&hiwdg);
	}

	uint32_t counter_led = 0;

    while(1) {


    	if (heartbeat_Sequencer == true && heartbeat_Komunikasi == true) {

			// Sistem Sehat Total! Tendang Watchdog Hardware
			HAL_IWDG_Refresh(&hiwdg);

	        // --- 2. KEDIP LED INDIKATOR SISTEM SEHAT ---
	        counter_led++;
	        // Task berjalan tiap 100ms. 100ms x 10 = 1000ms (1 Detik)
	        if (counter_led >= 10) {
	            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	            counter_led = 0;
	        }

			// Turunkan kembali bendera absen untuk ujian di siklus berikutnya
			heartbeat_Sequencer = false;
			heartbeat_Komunikasi = false;

		}

        // --- 1. UPDATE REGISTER MODBUS (MONITORING POSISI) ---
		int32_t currentX_pulse = motorX.getCurrentPositionPulse();
		int32_t currentY_pulse = motorY.getCurrentPositionPulse();
		int32_t currentZ_pulse = motorZ.getCurrentPositionPulse();

		// KONVERSI KE MILIMETER DENGAN PRESISI x100
		// Rumus: (Pulse * 100) / Steps_per_mm
		// Jika stepsPerMm bernilai 0, langsung jadikan hasilnya 0.
		// Jika tidak 0, baru lakukan operasi pembagian.

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



        // Tidurkan task selama 50 milidetik
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


