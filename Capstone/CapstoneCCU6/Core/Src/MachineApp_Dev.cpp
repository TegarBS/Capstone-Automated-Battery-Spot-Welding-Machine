/*
 * MachineApp_Dev.cpp
 *
 *  Created on: Jul 25, 2026
 *      Author: Tegar
 */

#include "MachineApp.h"
#include "StepperMotor.h"
#include "main.h"
#include "Communication.h"
#include "GlobalData.h"
#include "UtilitasDanPerhitungan.h"

#include "FungsiEEPROM.h"
#include "Monitor.h"

#define SETTLING_TIME_MS 100

void Task_Sequencer(void *pvParameters);

bool Wait_Motors_Reach_Target(bool includeZ);
void Handle_Emergency_Abort(void);
void Perform_Homing_Sequence(void);
void Perform_Auto_Weld_Cycle(void);
void Perform_Manual_Jog(void);
bool Eksekusi_Las(void);

StepperMotor motorX(STEP_X_GPIO_Port, STEP_X_Pin, DIR_X_GPIO_Port, DIR_X_Pin, &htim2);
StepperMotor motorY(STEP_Y_GPIO_Port, STEP_Y_Pin, DIR_Y_GPIO_Port, DIR_Y_Pin, &htim3);
StepperMotor motorZ(STEP_Z_GPIO_Port, STEP_Z_Pin, DIR_Z_GPIO_Port, DIR_Z_Pin, &htim4);

// --- FUNGSI INISIALISASI MESIN ---
void MachineApp_Init(void) {
	// -----------------------------------------------------
	// DIAGNOSTIK PENYEBAB RESET (PRE-FLIGHT CHECK)
	// -----------------------------------------------------
	// 1. Cek apakah mesin baru saja di-reset oleh Watchdog
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
		kode_alarm_sistem = 1; // Set alarm IWDG!
	}
	// 2. Cek apakah mesin di-reset oleh tombol fisik (Pin Reset)
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
		kode_alarm_sistem = 2;
	}
	// 3. Cek apakah mesin baru saja dicolok listrik (Power On Reset)
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
		kode_alarm_sistem = 3;
	}

	// WAJIB DILAKUKAN: Hapus bendera reset di memori hardware,
	// jika tidak, status IWDG akan terus terbaca di reset berikutnya!
	__HAL_RCC_CLEAR_RESET_FLAGS();

    // Buat Semaphore & Queue
    SinyalPosisiTercapai = xSemaphoreCreateBinary();
    Sem_DataDiterima = xSemaphoreCreateBinary();
    Sem_MintaSave = xSemaphoreCreateBinary();
    Sem_WeldDone = xSemaphoreCreateBinary();
    Sem_I2C_Rx = xSemaphoreCreateBinary();

    // Buat Task RTOS
    xTaskCreate(Task_Sequencer, "Task_Seq", 512, NULL, 2, NULL);
    xTaskCreate(Task_Komunikasi, "Task_Komunikasi", 512, NULL, 2, NULL);
    xTaskCreate(Task_Monitor, "Task_Mon", 256, NULL, 1, NULL);
    xTaskCreate(Task_Storage, "Task_Storage", 256, NULL, 1, NULL);
}

void MachineApp_Loop(void) {}

void Task_Sequencer(void *pvParameters) {
	Delay_With_Watchdog(500, heartbeat_Sequencer); // Jeda inisialisasi hardware

	//Menyalakan mesin las mode manual
	HAL_GPIO_WritePin(WELD_PIN_GPIO_Port, WELD_PIN_Pin, GPIO_PIN_SET);
	Delay_With_Watchdog(500, heartbeat_Sequencer);
	HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_SET);
	Delay_With_Watchdog(2000, heartbeat_Sequencer);
	HAL_GPIO_WritePin(WELD_PIN_GPIO_Port, WELD_PIN_Pin, GPIO_PIN_RESET);

	//Menunggu data konfigurasi dari Task Storage
    while(!isConfigLoaded){
        vTaskDelay(pdMS_TO_TICKS(10));
        heartbeat_Sequencer = true;
        //status = meunggu data konfig
    }

	HAL_GPIO_WritePin(STEP_X_GPIO_Port, STEP_X_Pin , GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_Y_GPIO_Port, STEP_Y_Pin , GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_Z_GPIO_Port, STEP_Z_Pin , GPIO_PIN_SET);

    while(1) {
        if (isAborted) {
            Handle_Emergency_Abort();
        }
        else if (isPerformHoming) {
            Perform_Homing_Sequence();
        }
        else if (isAutoWeldActive) {
            Perform_Auto_Weld_Cycle();
        }
        else if (isPerformManualJog) {
            Perform_Manual_Jog();
        }
        else if (isPerformWeld && !isAutoWeldActive && !isNavigating) {
    		if (!Eksekusi_Las()) {
    			// TERJADI ERROR PNEUMATIK!
    			isAborted = true;
    			continue;             // Hentikan seluruh mesin
    			//Kirim_Pesan_Error_Ke_HMI();   // Beritahu layar Haiwell
    		}
    		isPerformWeld = false;
    		isBusy = false;
        }
		else if (isPerformPneuUp) {
			// 1. RISING EDGE
			isBusy = true;
			Silinder_Up();

			while (isPerformPneuUp) {
				vTaskDelay(pdMS_TO_TICKS(10)); // WAJIB ADA DELAY![cite: 6]
				heartbeat_Sequencer = true;
				if (isAborted) break;
			}

			// 3. FALLING EDGE
			if (!isAborted) {
				Silinder_Hold();
				isBusy = false;
			}
		}
        else if (isPerformPneuDown) {
			// 1. RISING EDGE (Eksekusi saat HMI mulai menekan)
        	isBusy = true;
			Silinder_Down();

			// 2. MENUNGGU (Tahan posisi selama flag masih TRUE)
			while (isPerformPneuDown) {
				// WAJIB pakai delay agar CPU bisa mengeksekusi Task_Komunikasi
				vTaskDelay(pdMS_TO_TICKS(10));
				heartbeat_Sequencer = true;    // Absen Watchdog IWDG

				if (isAborted) break;          // Sabuk pengaman E-Stop
			}

			// 3. FALLING EDGE (Eksekusi seketika saat HMI melepas tekanan)
			if (!isAborted) {
				Silinder_Hold();
				isBusy = false;
			}
		}


        else {
            // =========================================================
            // E. MODE STANDBY PURE (Menunggu Perintah)
            // =========================================================
            vTaskDelay(pdMS_TO_TICKS(50));
            heartbeat_Sequencer = true; // ABSENSI WAJIB
        }
    }
}


// ---------------------------------------------------------
// HELPER 1: Menunggu Motor Selesai Bergerak (Aman dari Watchdog & E-Stop)
// Return TRUE jika sukses sampai tujuan, FALSE jika dibatalkan (E-Stop)
// ---------------------------------------------------------
bool Wait_Motors_Reach_Target(bool includeZ = false) {
    while (motorX.isMoving() || motorY.isMoving() || (includeZ && motorZ.isMoving())) {
        xSemaphoreTake(SinyalPosisiTercapai, pdMS_TO_TICKS(20));
        heartbeat_Sequencer = true; // LAPOR MANDOR IWDG!

        if (isAborted) return false; // Tembus seketika jika E-Stop ditekan
    }
    return true;
}

void Handle_Emergency_Abort(void) {
	Set_Mesin_Error(403);

	isNavigating = false;

    isAutoWeldActive = false;
	isPerformHoming = false;
	isPerformManualJog = false;
	isPerformWeld = false;
	isPerformPneuUp = false;
	isPerformPneuDown = false;

	isBusy = false;
    isAborted = false;
    isMachineHomed = false;

    Silinder_Up();

    //WELD OFF ALL PIN
    HAL_GPIO_WritePin(WELD_PIN_GPIO_Port, WELD_PIN_Pin, GPIO_PIN_RESET);

    motorX.stop();
    motorY.stop();
    motorZ.stop();
}

bool Homing_Silinder_Z(void) {
    uint32_t startTime = 0;

    Silinder_Up();
    startTime = HAL_GetTick();

    while (HAL_GPIO_ReadPin(REED_TOP_GPIO_Port, REED_TOP_Pin) != REED_TOP_ON) {

        vTaskDelay(pdMS_TO_TICKS(1));

        if ((HAL_GetTick() - startTime) > eepromSettings.time_out_pneumatic_ms) {
            return false;
        }
        heartbeat_Sequencer = true;
    }
    Silinder_Hold();

    return true;
}

void Perform_Homing_Sequence(void) {
    Update_Motor_Param_Homing_From_Settings();

   if (!Homing_Silinder_Z()) {
	   isAborted = true;
	   Set_Mesin_Error(301);
	   return;
   }

    // --- TAHAP 1: SEEK ---
    isNavigating = true;
    motorX.isHomed = false;
    motorY.isHomed = false;

    if (HAL_GPIO_ReadPin(LIMIT_X_GPIO_Port, LIMIT_X_Pin) == LIMIT_X_Pressed) motorX.stop();
    else motorX.setTargetPulse(-999999);

    if (HAL_GPIO_ReadPin(LIMIT_Y_GPIO_Port, LIMIT_Y_Pin) == LIMIT_Y_Pressed) motorY.stop();
    else motorY.setTargetPulse(-999999);

    if (!Wait_Motors_Reach_Target()) return;

    motorX.resetPosition();
    motorY.resetPosition();

    // --- TAHAP 2: PULL-OFF ---
    ignoreLimitSwitch = true;
    motorX.setTargetPulse(Convert_mm_to_PulseX(eepromSettings.homingTravelAfterLimitX));
    motorY.setTargetPulse(Convert_mm_to_PulseY(eepromSettings.homingTravelAfterLimitY));

    if (!Wait_Motors_Reach_Target()) return;

    ignoreLimitSwitch = false;

    // --- TAHAP 3: ZERO ABSOLUT ---
    motorX.resetPosition();
    motorY.resetPosition();
    Update_Motor_Param_From_Settings();

    isMachineHomed = true;

    isPerformHoming = false;
    isBusy = false;
}

void Perform_Auto_Weld_Cycle(void) {

    if (!isMachineHomed) {
        isPerformHoming = true; return;
    }

	if (isFirstWeldCycle) {

		isBusy = true;

		if(HAL_GPIO_ReadPin(REED_TOP_GPIO_Port, REED_TOP_Pin) != REED_TOP_ON){
		   if (!Homing_Silinder_Z()) {
			   isAborted = true;
			   Set_Mesin_Error(301);
			   return;
		   }
		}

		if (configLas.startKolom > configLas.totalKolom || configLas.startBaris > configLas.totalBaris) {
			indexKolomAktual = 0;
			indexBarisAktual = 0;
			Set_Mesin_Error(501);
		} else {
			indexKolomAktual = configLas.startKolom;
			indexBarisAktual = configLas.startBaris;
		}

		if((configLas.totalKolom * configLas.pitchX) > eepromSettings.maxTravelX || (configLas.totalBaris * configLas.pitchY) > eepromSettings.maxTravelY){
			Set_Mesin_Error(502);
			isAutoWeldActive = false;   // 2. Batalkan mode Auto
			isAborted = true;
		}

		isFirstWeldCycle = false;
	}

    if (indexBarisAktual >= configLas.totalBaris) {
        isAutoWeldActive = false;
        isBusy = false;
        isSaveOdo = true;
        return;
    }

    while (isPaused && !isAborted) {
        heartbeat_Sequencer = true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (isAborted) return;

    Koordinat_t center = Calculate_Battery_Center_mm();

    // MINI-LOOP
    for (uint16_t i = 0; i < eepromSettings.weldCount; i++) {
        float offsetY_mm = 0.0f;
        if (eepromSettings.weldCount > 1) {
            float spacing = (float)eepromSettings.spotSpacing_x10 / 10.0f;
            offsetY_mm = -(spacing * (eepromSettings.weldCount - 1) / 2.0f) + (i * spacing);
        }

        isNavigating = true;
        motorX.setTargetPulse(Convert_mm_to_PulseX(center.x));
        motorY.setTargetPulse(Convert_mm_to_PulseY(center.y + offsetY_mm));

        if (!Wait_Motors_Reach_Target()) return; // Gagal / Batal

        isNavigating = false;

        Delay_With_Watchdog(200, heartbeat_Sequencer);

        mesinOdo.totalJarakXmm+=center.x;
        mesinOdo.totalJarakYmm+=center.y + offsetY_mm;

        isPerformWeld = true;

		if (!Eksekusi_Las()) {
			// TERJADI ERROR PNEUMATIK!
			isAborted = true;
			return;
		}

        mesinOdo.totalTitikLas++;

        //Delay_With_Watchdog(500, heartbeat_Sequencer);

        isPerformWeld = false;
    }

	// 5. UPDATE INDEX BATERAI SELANJUTNYA (ZIG-ZAG FISIK)
	if (indexBarisAktual % 2 == 0) {
		// ---------------------------------------------------
		// BARIS GENAP (0, 2, 4...): Mesin bergerak Kiri -> Kanan
		// ---------------------------------------------------
		if (indexKolomAktual < configLas.totalKolom - 1) {
			indexKolomAktual++; // Lanjut bergeser ke kanan
		} else {
			// Sudah mentok di ujung KANAN
			indexBarisAktual++; // Pindah baris (turun ke sumbu Y)
			// PENTING: indexKolomAktual TIDAK di-nol-kan. Ia tetap di ujung kanan!
		}

	} else {

		// ---------------------------------------------------
		// BARIS GANJIL (1, 3, 5...): Mesin bergerak Kanan -> Kiri
		// ---------------------------------------------------
		if (indexKolomAktual > 0) {
			indexKolomAktual--; // Lanjut bergeser ke kiri (Mundur)
		} else {
			// Sudah mentok di ujung KIRI
			indexBarisAktual++; // Pindah baris (turun ke sumbu Y)
			// PENTING: indexKolomAktual sudah bernilai 0 (ujung kiri). Biarkan saja!
		}

	}

	isPerformWeld = false;
}

void Perform_Manual_Jog(void) {
	int16_t jogManX_mm = (int16_t)Holding_Registers[6];
	int16_t jogManY_mm = (int16_t)Holding_Registers[8];
	int16_t jogManZ_mm = (int16_t)Holding_Registers[10];

	int32_t targetAbsX = motorX.getCurrentPositionPulse() + Convert_mm_to_PulseX(jogManX_mm);
	int32_t targetAbsY = motorY.getCurrentPositionPulse() + Convert_mm_to_PulseY(jogManY_mm);
	int32_t targetAbsZ = motorZ.getCurrentPositionPulse() + Convert_mm_to_PulseZ(jogManZ_mm);

	if(targetAbsX > Convert_mm_to_PulseX(eepromSettings.maxTravelX)) Set_Mesin_Error(206);
	if(targetAbsY > Convert_mm_to_PulseX(eepromSettings.maxTravelY)) Set_Mesin_Error(207);
	if(targetAbsZ > Convert_mm_to_PulseX(eepromSettings.maxTravelZ)) Set_Mesin_Error(208);

    isNavigating = true;
    motorX.setTargetPulse(targetAbsX);
    motorY.setTargetPulse(targetAbsY);
    motorZ.setTargetPulse(targetAbsZ);

    if (!Wait_Motors_Reach_Target(true)) return;

    isNavigating = false;
    isPerformManualJog = false;
    isBusy = false;

    // Reset Register
    Holding_Registers[6] = 0;
	Holding_Registers[8] = 0;
	Holding_Registers[10] = 0;
}

bool Eksekusi_Las(void) {
	uint32_t startTime;
    // ==========================================================
    // 1. SILINDER TURUN
    // ==========================================================
    Silinder_Down();
    startTime = HAL_GetTick();

    if (isAborted) return false;

    // ==========================================================
    // 2. MENUNGGU REED SWITCH BAWAH TERSENTUH
    // ==========================================================
    while (HAL_GPIO_ReadPin(REED_BOTTOM_GPIO_Port, REED_BOTTOM_Pin) != REED_BOTTOM_ON) {

        // Tidurkan Task 1 milidetik agar CPU bisa mengurus komunikasi HMI & Task Lain
        vTaskDelay(pdMS_TO_TICKS(1));

        // Pengecekan Timeout (Jika macet lebih dari 2 detik)
        if ((HAL_GetTick() - startTime) > eepromSettings.time_out_pneumatic_ms) {
            Silinder_Up();// Paksa naikkan silinder
            Set_Mesin_Error(302);
            return false;
        }

        // Tendang Watchdog agar tidak reset saat menunggu
        heartbeat_Sequencer = true;
    }

    if (isAborted) return false;

    // [TIPS INDUSTRI]: Beri jeda sedikit setelah sensor tersentuh agar
    // tekanan pneumatik benar-benar padat ke nikel baterai sebelum arus listrik ditembakkan.
    vTaskDelay(pdMS_TO_TICKS(SETTLING_TIME_MS));

    // ==========================================================
    // 3. PERFORM LAS (PENGGANTI PEDAL)
    // ==========================================================
    uint32_t main_weld_us = (uint32_t)eepromSettings.main_pulse_weld_time * 1000;

    if (main_weld_us > 0) {
		// Kuras isi Semaphore lagi
		xSemaphoreTake(Sem_WeldDone, 0);

		__HAL_TIM_SET_AUTORELOAD(&htim5, main_weld_us);
		__HAL_TIM_SET_COUNTER(&htim5, 0);
		HAL_GPIO_WritePin(WELD_PIN_GPIO_Port, WELD_PIN_Pin, GPIO_PIN_SET);
		HAL_TIM_Base_Start_IT(&htim5);

		heartbeat_Sequencer = true;

		if (xSemaphoreTake(Sem_WeldDone, pdMS_TO_TICKS((main_weld_us / 1000) + 100)) == pdTRUE) {
			// Normal
			heartbeat_Sequencer = true;
		} else {
			// FATAL ERROR
			heartbeat_Sequencer = true;
			HAL_GPIO_WritePin(WELD_PIN_GPIO_Port, WELD_PIN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ENABLE_PIN_GPIO_Port, ENABLE_PIN_Pin, GPIO_PIN_RESET);
			HAL_TIM_Base_Stop_IT(&htim5);
			Set_Mesin_Error(303);
			return false;
		}
    }

    // ==========================================================
    // 4. SILINDER NAIK
    // ==========================================================
    Silinder_Up();
    startTime = HAL_GetTick();

    // ==========================================================
    // 5. MENUNGGU REED SWITCH ATAS TERSENTUH (POSISI AMAN)
    // ==========================================================
    while (HAL_GPIO_ReadPin(REED_TOP_GPIO_Port, REED_TOP_Pin) != REED_TOP_ON) {

        vTaskDelay(pdMS_TO_TICKS(1));

        if ((HAL_GetTick() - startTime) > eepromSettings.time_out_pneumatic_ms) {
            //kode_alarm_sistem = 102; // Contoh: Error Silinder Gagal Naik
        	Set_Mesin_Error(301);
            return false; // Berhenti dan lapor error
        }

        heartbeat_Sequencer = true;
    }

    Silinder_Up();

    // Jika sampai di sini, siklus selesai dengan sempurna!
    return true;
}
