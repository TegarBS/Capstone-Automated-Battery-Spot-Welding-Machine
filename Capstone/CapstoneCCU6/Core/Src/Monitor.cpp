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
        if (motorX && motorY && motorZ) {
            long currentX_pulse = motorX->getCurrentPositionPulse();
            long currentY_pulse = motorY->getCurrentPositionPulse();
            long currentZ_pulse = motorZ->getCurrentPositionPulse();

            // KONVERSI KE MILIMETER DENGAN PRESISI x100
            // Rumus: (Pulse * 100) / Steps_per_mm
            long currentX_mm = (currentX_pulse * 100) / eepromSettings.stepsPerMmX;
            long currentY_mm = (currentY_pulse * 100) / eepromSettings.stepsPerMmY;
            long currentZ_mm = (currentZ_pulse * 100) / eepromSettings.stepsPerMmZ;

            // Pecah 32-bit (long) menjadi 2 bagian 16-bit untuk Input Register Modbus
            Input_Registers[0] = currentX_mm & 0xFFFF;
            Input_Registers[1] = (currentX_mm >> 16) & 0xFFFF;
            Input_Registers[2] = currentY_mm & 0xFFFF;
            Input_Registers[3] = (currentY_mm >> 16) & 0xFFFF;
            Input_Registers[4] = currentZ_mm & 0xFFFF;
            Input_Registers[5] = (currentZ_mm >> 16) & 0xFFFF;

            Input_Registers[7] = indexKolomAktual + 1;
            Input_Registers[8] = indexBarisAktual + 1;

            Input_Registers[9] = kode_alarm_sistem;
        }



        // Tidurkan task selama 50 milidetik
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


