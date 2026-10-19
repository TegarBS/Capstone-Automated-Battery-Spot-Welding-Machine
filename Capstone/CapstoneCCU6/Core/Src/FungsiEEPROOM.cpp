/*
 * FungsiEEPROOM.cpp
 *
 *  Created on: Jul 30, 2026
 *      Author: Tegar
 */

#include "FungsiEEPROM.h"
#include "GlobalData.h"
#include <string.h>
#include "UtilitasDanPerhitungan.h"

uint16_t eeprom=0;

extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;

// Alamat default AT24C256 (A0, A1, A2 terhubung ke GND)
// STM32 menggunakan format 8-bit, jadi 0x50 digeser 1 bit kiri menjadi 0xA0
#define EEPROM_I2C_ADDR 0xA0

// Kavling Memori
#define ADDR_ODOMETER   0x0000  // Mulai dari byte 0
#define ADDR_SETTINGS   0x0040  // Mulai dari byte 64

#define MAX_ODO_SLOTS      5         // 32 Byte ruang / 4 Byte data = 8 Slot
#define WEAR_LEVEL_LIMIT   100000    // Pindah slot setiap 100 ribu titik las

#define MAGIC_NUMBER 0xABCD1234      // Kode rahasia penanda Flash valid

#define IC_ADDRESS_SIZE I2C_MEMADD_SIZE_16BIT

void Task_Storage(void *pvParameters) {
	vTaskDelay(pdMS_TO_TICKS(100));

	if (Load_Settings_From_EEPROM()) {
		// Fisik sukses, sekarang cek validitas datanya
		if (eepromSettings.magicNumber == MAGIC_NUMBER) {
			// DATA VALID & SUKSES! Mesin siap digunakan dengan setelan terakhir.
			//Set_Mesin_Error(100);
		} else {
			// Data ditarik dengan sukses, TAPI isinya kosong (EEPROM baru/corrupt)
			Set_Mesin_Error(101);
			Load_Default_Value();
			//Save_Settings_To_EEPROM(); // Tulis nilai default ke EEPROM agar besok valid
		}

	} else {
		// Gagal fisik (Kabel I2C putus, atau chip EEPROM rusak)
		Set_Mesin_Error(102);
		Load_Default_Value(); // Beri nilai aman agar mesin tidak HardFault
	}

	Salin_Data_Ke_Register();
	Update_Motor_Param_From_Settings();

	isConfigLoaded = true;

	if (Load_Odometer_From_EEPROM()) {
	} else {
		Set_Mesin_Error(104);
	}

	eeprom++;

    while(1) {
        // Tidur pulas sampai ada yang menyuruh Save
        if (xSemaphoreTake(Sem_MintaSave, pdMS_TO_TICKS(50)) == pdTRUE) {

        	eeprom+=100;
            // Lakukan penulisan 300ms
            Ambil_Data_Dari_Holding_Register();
            Update_Motor_Param_From_Settings();

        	if (Save_Settings_To_EEPROM()) {
        		//Berhasil menyimpan konfig
        	} else {
        		//I2C fail
        		Set_Mesin_Error(103);
        	}

        	isBusy = false;

        }else if(isSaveOdo){
        	if (Save_Odometer_To_EEPROM()) {
        	} else {
        		Set_Mesin_Error(105);
        	}

        	isSaveOdo = false;
        }
    }
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // Bangunkan Task yang sedang menunggu xSemaphoreTake di fungsi Load
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(Sem_I2C_Rx, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// --- FUNGSI MEMBACA DARI FLASH ---
bool Load_Settings_From_EEPROM(void) {
    uint8_t *pData = (uint8_t*)&eepromSettings;
    uint16_t ukuran_struct = sizeof(PermanentSettings_t);

    // 1. Kuras Semaphore agar tidak ada "sinyal hantu"
    xSemaphoreTake(Sem_I2C_Rx, 0);

    // 2. Tembakkan perintah tarik data via DMA
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(
        &hi2c1, EEPROM_I2C_ADDR,
        ADDR_SETTINGS, IC_ADDRESS_SIZE,
        pData, ukuran_struct
    );

    // Jika bus I2C sibuk atau kabel putus sejak awal
    if (status != HAL_OK) {
        return false;
    }

    // 3. Tidurkan Task RTOS maksimal 1 detik menunggui DMA
    if (xSemaphoreTake(Sem_I2C_Rx, pdMS_TO_TICKS(1000)) == pdTRUE) {
        return true; // Transmisi I2C Sukses! (Isi datanya belum divalidasi)
    }

    // Jika sampai baris ini, terjadi Timeout (IC mati / kabel putus di jalan)
    return false;
}

bool Save_Settings_To_EEPROM(void) {
    // 1. Safety Interlock
    if (isAutoWeldActive || isPerformManualJog || motorX.isMoving()) {
        return false; // Mesin sedang sibuk
    }

    // 3. Konversi struct menjadi deretan byte (array)
    uint8_t *pData = (uint8_t*)&eepromSettings;
    uint16_t ukuran_struct = sizeof(PermanentSettings_t);

    // 4. Tulis Byte per Byte secara aman
    for (uint16_t i = 0; i < ukuran_struct; i++) {

        // Parameter: Handle I2C, Alamat IC, Alamat Memori Internal (16-BIT!), Ukuran Alamat, Data, Panjang, Timeout
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
            &hi2c1, EEPROM_I2C_ADDR,
            ADDR_SETTINGS + i, IC_ADDRESS_SIZE,
            &pData[i], 1, 100
        );

        if (status != HAL_OK) {
            return false; // Terjadi error pada kabel I2C
        }

        // IC EEPROM butuh waktu fisik maksimal 5ms untuk "memanggang" 1 byte data
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return true; // Sukses menyimpan tanpa mereset mesin!
}

void Salin_Data_Ke_Register(void){
	memcpy(&Holding_Registers[EEPROM_START_MODBUS_REG],
		   &eepromSettings.stepsPerMmX,
		   TOTAL_EEPROM_PARAMS * sizeof(uint16_t));
}

void Ambil_Data_Dari_Holding_Register(void){
	eepromSettings.magicNumber = MAGIC_NUMBER;

	memcpy(&eepromSettings.stepsPerMmX,
		   &Holding_Registers[EEPROM_START_MODBUS_REG],
		   TOTAL_EEPROM_PARAMS * sizeof(uint16_t));
}

bool Load_Odometer_From_EEPROM(void) {
    // 1. Buffer sekarang menggunakan cetakan Struct, bukan uint32_t
    Odometer_t bufferOdo[MAX_ODO_SLOTS];
    uint16_t total_ukuran_baca = sizeof(bufferOdo); // 5 x 12 = 60 Byte

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1, EEPROM_I2C_ADDR,
        ADDR_ODOMETER, IC_ADDRESS_SIZE,
        (uint8_t*)bufferOdo, total_ukuran_baca, 1000
    );

    if (status != HAL_OK) {
        mesinOdo.totalTitikLas = 0;
        mesinOdo.totalJarakXmm = 0;
        mesinOdo.totalJarakYmm = 0;
        return false;
    }

    // 2. ALGORITMA PENCARIAN BARU (Fokus pada totalTitikLas)
    uint32_t nilai_tertinggi = 0;
    uint8_t index_terbaru = 0; // Mengingat di slot mana data terbaru berada

    for(int i = 0; i < MAX_ODO_SLOTS; i++) {
        // Cek jika slot tidak kosong (0xFFFFFFFF) dan totalTitikLas lebih besar
        if (bufferOdo[i].totalTitikLas != 0xFFFFFFFF && bufferOdo[i].totalTitikLas > nilai_tertinggi) {
            nilai_tertinggi = bufferOdo[i].totalTitikLas;
            index_terbaru = i;
        }
    }

    // 3. Salin seluruh isi struct dari slot yang paling baru
    mesinOdo = bufferOdo[index_terbaru];

    return true;
}

bool Save_Odometer_To_EEPROM(void) {
    uint8_t *pData = (uint8_t*)&mesinOdo;
    uint16_t ukuran_struct = sizeof(Odometer_t); // 4 Byte

    // 1. Tentukan Slot Aktif berdasarkan total las
    // Contoh: 150.000 / 100.000 = 1. (1 % 8) = Slot 1
    // Contoh: 850.000 / 100.000 = 8. (8 % 8) = Slot 0 (Kembali ke awal)
    uint8_t slot_aktif = (mesinOdo.totalTitikLas / WEAR_LEVEL_LIMIT) % MAX_ODO_SLOTS;

    // 2. Kalkulasi alamat fisik di EEPROM
    uint16_t alamat_target = ADDR_ODOMETER + (slot_aktif * ukuran_struct);

    // 3. Tulis per byte secara aman ke alamat yang sudah digeser
    for (uint16_t i = 0; i < ukuran_struct; i++) {
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
            &hi2c1, EEPROM_I2C_ADDR,
            alamat_target + i, IC_ADDRESS_SIZE,
            &pData[i], 1, 100
        );

        if (status != HAL_OK) return false;

        vTaskDelay(pdMS_TO_TICKS(5)); // Waktu panggang EEPROM
    }

    return true;
}

bool Reset_Odometer_EEPROM(void) {
    // 1. Reset nilai di memori RAM (SRAM) terlebih dahulu
    mesinOdo.totalTitikLas = 0;
    mesinOdo.totalJarakXmm = 0;
    mesinOdo.totalJarakYmm = 0;

    uint8_t *pData = (uint8_t*)&mesinOdo;
    uint16_t ukuran_struct = sizeof(Odometer_t); // 12 Byte

    // 2. Lakukan perulangan untuk menimpa SELURUH SLOT (5 Slot)
    for (uint8_t slot = 0; slot < MAX_ODO_SLOTS; slot++) {

        // Hitung alamat fisik untuk masing-masing slot
        uint16_t alamat_target = ADDR_ODOMETER + (slot * ukuran_struct);

        // 3. Tulis Byte-by-Byte secara aman untuk slot ini
        for (uint16_t i = 0; i < ukuran_struct; i++) {
            HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
                &hi2c1, EEPROM_I2C_ADDR,
                alamat_target + i, I2C_MEMADD_SIZE_16BIT,
                &pData[i], 1, 100
            );

            if (status != HAL_OK) return false;

            // Waktu panggang EEPROM fisik maksimal 5ms per byte
            vTaskDelay(pdMS_TO_TICKS(5));

        }
    }

    return true;
}


void Load_Default_Value(void){
    eepromSettings.magicNumber = MAGIC_NUMBER;

    eepromSettings.stepsPerMmX = 80;
    eepromSettings.stepsPerMmY = 107;
    eepromSettings.stepsPerMmZ = 800;

    //Speed in mm/s
    eepromSettings.maxSpeedX = 2500;
    eepromSettings.maxSpeedY = 2500;
    eepromSettings.maxSpeedZ = 500;

    eepromSettings.minSpeedX = 500;
    eepromSettings.minSpeedY = 500;
    eepromSettings.minSpeedZ = 400;

    //Acceleration in mm/s^2
    eepromSettings.accelX = 1000;
    eepromSettings.accelY = 1000;
    eepromSettings.accelZ = 50;

    //Max travel in mm
    eepromSettings.maxTravelX = 200;
    eepromSettings.maxTravelY = 200;
    eepromSettings.maxTravelZ = 50;

    //Speed in mm/s
    eepromSettings.maxSpeedHomingX = 1500;
    eepromSettings.maxSpeedHomingY = 1500;
    eepromSettings.maxSpeedHomingZ = 500;

    //Travel in mm
    eepromSettings.homingTravelAfterLimitX = 20;
    eepromSettings.homingTravelAfterLimitY = 20;
    eepromSettings.homingTravelAfterLimitZ = 20;

    eepromSettings.directionX = 0;
    eepromSettings.directionY = 0;
    eepromSettings.directionZ = 0;

    eepromSettings.weldCount = 2;
    eepromSettings.spotSpacing_x10 = 15;

    eepromSettings.X_distance_from_0 = 90;
    eepromSettings.Y_distance_from_0 = 90;

    eepromSettings.time_out_pneumatic_ms = 2000;

    eepromSettings.RESERVED_pre_heat_time_ms = 2;
    eepromSettings.RESERVED_interval_time_ms = 5;
    eepromSettings.main_pulse_weld_time = 100;

}
