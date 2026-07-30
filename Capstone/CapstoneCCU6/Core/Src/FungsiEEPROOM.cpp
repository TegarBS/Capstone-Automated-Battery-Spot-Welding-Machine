/*
 * FungsiEEPROOM.cpp
 *
 *  Created on: Jul 30, 2026
 *      Author: Tegar
 */

#include "FungsiEEPROM.h"
#include "GlobalData.h"

#define FLASH_EEPROM_ADDR 0x08020000 // Alamat awal Sektor 5 di STM32F401
#define MAGIC_NUMBER 0xABCD1234      // Kode rahasia penanda Flash valid

// --- FUNGSI MEMBACA DARI FLASH ---
void Load_Settings_From_Flash() {
    // Arahkan pointer untuk membaca langsung dari alamat Flash hardware
    PermanentSettings_t *flashData = (PermanentSettings_t*)FLASH_EEPROM_ADDR;

    if (flashData->magicNumber == MAGIC_NUMBER) {
        // Jika kode cocok, copy isi Flash ke RAM
        eepromSettings = *flashData;
    } else {
        // Jika Flash masih kosong / pabrikan, beri nilai Default!
        eepromSettings.magicNumber = MAGIC_NUMBER;
        eepromSettings.stepsPerMmX = 160;
        eepromSettings.stepsPerMmY = 160;
        eepromSettings.maxSpeed = 2000;
        eepromSettings.accelSpeed = 500;
    }

    // Salin ke Modbus Holding Registers agar bisa dibaca oleh HMI saat konek pertama kali
    Holding_Registers[10] = eepromSettings.stepsPerMmX;
    Holding_Registers[11] = eepromSettings.stepsPerMmY;
    Holding_Registers[12] = eepromSettings.maxSpeed;
    Holding_Registers[13] = eepromSettings.accelSpeed;
}

// --- FUNGSI MENYIMPAN KE FLASH (DENGAN SAFETY INTERLOCK) ---
bool Save_Settings_To_Flash() {

    // ==========================================================
    // 1. SAFETY INTERLOCK (PENCEGAHAN CRASH)
    // Jangan izinkan penyimpanan jika mesin sedang melakukan aktivitas kritis!
    // ==========================================================
    if (isAutoWeldActive || flagManualWeld || flagMintaHoming || isNavigating) {
        return false; // TOLAK PERINTAH! Mesin sedang sibuk
    }

    if (motorX && motorY) {
        if (motorX->isMoving || motorY->isMoving) {
            return false; // TOLAK PERINTAH! Motor sedang berputar
        }
    }

    // 2. Ambil data terbaru dari Holding Registers HMI
    eepromSettings.magicNumber = MAGIC_NUMBER;
    eepromSettings.stepsPerMmX = Holding_Registers[10];
    eepromSettings.stepsPerMmY = Holding_Registers[11];
    eepromSettings.maxSpeed    = Holding_Registers[12];
    eepromSettings.accelSpeed  = Holding_Registers[13];

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
    uint32_t *pRamData = (uint32_t*)&eepromSettings;
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
