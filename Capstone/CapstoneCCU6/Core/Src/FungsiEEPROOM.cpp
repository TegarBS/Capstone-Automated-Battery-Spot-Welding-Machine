/*
 * FungsiEEPROOM.cpp
 *
 *  Created on: Jul 30, 2026
 *      Author: Tegar
 */

#include "FungsiEEPROM.h"
#include "GlobalData.h"
#include <string.h>

#define FLASH_EEPROM_ADDR 0x08020000 // Alamat awal Sektor 5 di STM32F401
#define MAGIC_NUMBER 0xABCD1234      // Kode rahasia penanda Flash valid

// --- FUNGSI MEMBACA DARI FLASH ---
void Load_Settings_From_Flash(void) {
    // Arahkan pointer untuk membaca langsung dari alamat Flash hardware
    const PermanentSettings_t *flashData = (const PermanentSettings_t*)FLASH_EEPROM_ADDR;

    if (flashData->magicNumber == MAGIC_NUMBER) {
        // Jika kode cocok, copy isi Flash ke RAM
        eepromSettings = *flashData;
    } else {
        // Jika Flash masih kosong / pabrikan, beri nilai Default!
    	Load_Default_Value();
    }

    // Salin ke Modbus Holding Registers agar bisa dibaca oleh HMI saat konek pertama kali
    Salin_Data_Ke_Register();
}

// --- FUNGSI MENYIMPAN KE FLASH (DENGAN SAFETY INTERLOCK) ---
bool Save_Settings_To_Flash(void) {

    // ==========================================================
    // 1. SAFETY INTERLOCK (PENCEGAHAN CRASH)
    // Jangan izinkan penyimpanan jika mesin sedang melakukan aktivitas kritis!
    // ==========================================================
    if (isAutoWeldActive || flagManualJog || flagMintaHoming || isNavigating) {
        return false; // TOLAK PERINTAH! Mesin sedang sibuk
    }

    if (motorX && motorY) {
        if (motorX->isMoving() || motorY->isMoving()) {
            return false; // TOLAK PERINTAH! Motor sedang berputar
        }
    }

    // 2. Ambil data terbaru dari Holding Registers HMI
    Ambil_Data_Dari_Holding_Register();

    // 3. MATIKAN SELURUH AKTIVITAS HARDWARE (GLOBAL DISABLE)
    __disable_irq();

    // 4. Buka gembok Flash STM32
    HAL_FLASH_Unlock();

    // 5. Hapus Sektor 5 (Wajib sebelum menulis)
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_5;
    EraseInitStruct.NbSectors = 1;
    HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

    // 6. Tulis ulang data Struct ke Flash (Kata per Kata / 32-bit)
    const uint32_t *pRamData = reinterpret_cast<const uint32_t*>(&eepromSettings);
    uint32_t wordsToWrite = sizeof(PermanentSettings_t) / 4;
    if (sizeof(PermanentSettings_t) % 4 != 0) wordsToWrite++;

    for (uint32_t i = 0; i < wordsToWrite; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_EEPROM_ADDR + (i * 4), pRamData[i]);
    }

    // 7. Kunci kembali Flash
    HAL_FLASH_Lock();

    // 8. NYALAKAN KEMBALI SELURUH AKTIVITAS HARDWARE
    __enable_irq();

    return true; // Penyimpanan Berhasil
}

void Salin_Data_Ke_Register(void){
	memcpy(&Holding_Registers[EEPROM_START_MODBUS_REG],
		   &eepromSettings.stepsPerMmX,
		   TOTAL_EEPROM_PARAMS * sizeof(uint16_t));
}

void Ambil_Data_Dari_Holding_Register(void){
	eepromSettings.magicNumber = MAGIC_NUMBER;

	// Kopi 26 parameter sekaligus dari Array HMI masuk ke RAM mesin
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
}
