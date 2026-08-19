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
#define ADDR_SETTINGS   0x0020  // Mulai dari byte 32

#define MAGIC_NUMBER 0xABCD1234      // Kode rahasia penanda Flash valid


void Task_Storage(void *pvParameters) {
	vTaskDelay(pdMS_TO_TICKS(100));
	eeprom++;

    while(1) {
        // Tidur pulas sampai ada yang menyuruh Save
        if (xSemaphoreTake(Sem_MintaSave, portMAX_DELAY) == pdTRUE) {

        	eeprom+=100;
            // Lakukan penulisan 300ms
            Save_Settings_To_EEPROM();
            Ambil_Data_Dari_Holding_Register();
            Update_Motor_Param_From_Settings();


            // Jangan lupa reset IWDG di dalam fungsi Save-nya
        }
    }
}


// --- FUNGSI MEMBACA DARI FLASH ---
void Load_Settings_From_EEPROM(void) {

    uint8_t *pData = (uint8_t*)&eepromSettings;
    uint16_t ukuran_struct = sizeof(PermanentSettings_t);

    // 1. Tarik semua isi EEPROM sekaligus ke dalam Struct RAM sementara
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1, EEPROM_I2C_ADDR,
        ADDR_SETTINGS, I2C_MEMADD_SIZE_16BIT,
        pData, ukuran_struct, 1000
    );

    // 2. Cek apakah komunikasi kabel I2C sukses
    if (status != HAL_OK) {
        // Gagal membaca (kabel putus atau IC rusak)
        Load_Default_Value(); // Isi dengan default pabrik agar tidak HardFault
        Salin_Data_Ke_Register();
        return;
    }

    // 3. CEK MAGIC NUMBER (Apakah EEPROM masih kosong/baru dari toko?)
    // IC EEPROM baru dari pabrik biasanya berisi 0xFF di seluruh byte-nya.
    if (eepromSettings.magicNumber == MAGIC_NUMBER) {
        // DATA VALID! Magic Number cocok.
        // Data sudah otomatis masuk ke struct eepromSettings melalui pointer pData
    } else {
    	eeprom+=10;
        Load_Default_Value();
    }

    // 4. Sinkronisasi data ke HMI
    Salin_Data_Ke_Register();

    return;
}

bool Save_Settings_To_EEPROM(void) {
    // 1. Safety Interlock
    if (isAutoWeldActive || isPerformManualJog || motorX.isMoving()) {
        return false; // Mesin sedang sibuk
    }

    // 2. Ambil data terbaru dari HMI
    Ambil_Data_Dari_Holding_Register();

    // 3. Konversi struct menjadi deretan byte (array)
    uint8_t *pData = (uint8_t*)&eepromSettings;
    uint16_t ukuran_struct = sizeof(PermanentSettings_t);

    // 4. Tulis Byte per Byte secara aman
    for (uint16_t i = 0; i < ukuran_struct; i++) {

        // Parameter: Handle I2C, Alamat IC, Alamat Memori Internal (16-BIT!), Ukuran Alamat, Data, Panjang, Timeout
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
            &hi2c1, EEPROM_I2C_ADDR,
            ADDR_SETTINGS + i, I2C_MEMADD_SIZE_16BIT,
            &pData[i], 1, 100
        );

        if (status != HAL_OK) {
            return false; // Terjadi error pada kabel I2C
        }

        // IC EEPROM butuh waktu fisik maksimal 5ms untuk "memanggang" 1 byte data
        vTaskDelay(pdMS_TO_TICKS(5));

        // Beri napas buatan pada Watchdog yang ketat (150ms)
        //heartbeat_Sequencer = true;
        // Jika pakai HAL IWDG langsung: HAL_IWDG_Refresh(&hiwdg);
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
    eepromSettings.weld_time = 20;
}
