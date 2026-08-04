/*
 * Communication.cpp
 *
 *  Created on: Jul 31, 2026
 *      Author: Tegar
 */


#include "Communication.h"

#include "FungsiEEPROM.h"
#include "GlobalData.h"
#include "UtilitasDanPerhitungan.h"

// --- Konstanta Modbus ---
const uint16_t MODBUS_COIL_ON = 0xFF00;
const uint8_t  MODBUS_EXCEPTION_ILLEGAL_ADDRESS    = 0x02; // Alamat tidak ditemukan
const uint8_t  MODBUS_EXCEPTION_BUSY               = 0x06; // Perangkat sibuk
const uint8_t  MODBUS_EXCEPTION_ILLEGAL_FUNCTION   = 0x01; // Illegal Function
const uint8_t  MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE = 0x03;

// --- Daftar Alamat Coil ---
enum ModbusCoilAddress {
    COIL_START_AUTO_WELD = 0x0000,
    COIL_EMERGENCY_STOP  = 0x0001,
    COIL_REQUEST_HOMING  = 0x0002,
    COIL_PAUSE_RESUME    = 0x0003,
    COIL_MANUAL_JOG      = 0x0004,

    COIL_SAVE_TO_FLASH   = 0x00010,
    COIL_RESET_SYSTEM    = 0x00011,
	COIL_CMD_ZERO_Z      = 0x00012,
	COIL_CMD_UNLOCK_Z    = 0x00013,

	MAX_COILS_READABLE   = 10
};

enum FunctionCode {
    READ_COIL              = 0x01,
    READ_DISCRETE_INPUT    = 0x02,
    READ_HOLDING_REGISTER  = 0x03,
    READ_INPUT_REGISTER    = 0x04,
    WRITE_SINGLE_COIL      = 0x05,
    WRITE_SINGLE_REGISTER  = 0x06
};

// Kamus Alamat Modbus Discrete Inputs (Read-Only Hardware)
enum ModbusDiscreteInput {
    DI_LIMIT_SWITCH_X   = 0x0000,
    DI_LIMIT_SWITCH_Y   = 0x0001,
    DI_LIMIT_SWITCH_Z   = 0x0002,
	DI_IS_MACHINE_HOMED = 0x0003,

    MAX_DISCRETE_INPUTS = 4
};

// =========================================================
// TASK KOMUNIKASI (PARSER MODBUS RTU ZERO-QUEUE)
// =========================================================
void Task_Komunikasi(void *pvParameters) {
    // Definisi konstanta ID Slave agar tidak menggunakan magic number (0x01)
    const uint8_t SLAVE_ID = 0x01;

    while(1) {
        if (xSemaphoreTake(Sem_DataDiterima, pdMS_TO_TICKS(50)) == pdTRUE) {

            uint8_t len = rxIndex;
            rxIndex = 0; // Reset untuk paket berikutnya

            // 1. Validasi Slave ID
            if (rxBuffer[0] != SLAVE_ID) continue;

            // 2. Validasi CRC16
            uint16_t crc_hitung = Hitung_CRC16(rxBuffer, len - 2);
            uint16_t crc_diterima = (rxBuffer[len - 1] << 8) | rxBuffer[len - 2];
            if (crc_hitung != crc_diterima) continue;

            uint8_t function_code = rxBuffer[1];

            // 3. Modbus Function Router
            switch (function_code) {
				case READ_COIL:
					Function_Code_01_READ_COILS(rxBuffer, len);
					break;

				case READ_DISCRETE_INPUT:
					Function_Code_02_READ_DISCRETE_INPUT(rxBuffer, len);
					break;

				case READ_HOLDING_REGISTER:
					Function_Code_03_READ_HOLDING_REGISTER(rxBuffer, len);
					break;

				case READ_INPUT_REGISTER:
					Function_Code_04_READ_INPUT_REGISTER(rxBuffer, len);
					break;

                case WRITE_SINGLE_COIL:
                	Function_Code_05_WRITE_SINGLE_COIL(rxBuffer, len);
                    break;

                case WRITE_SINGLE_REGISTER:
                	Function_Code_06_WRITE_SINGLE_REGISTER(rxBuffer, len);
                    break;

                default:
                    Send_Modbus_Exception(rxBuffer[0], function_code, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
                    break;
            }
        }

        else{}
        heartbeat_Komunikasi = true;
    }
}

// =========================================================
// CALLBACK INTERUPSI UART (HARDWARE ISR)
// =========================================================
// cppcheck-suppress constParameterPointer
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        rxBuffer[rxIndex++] = rxChar;

        // Cegah overflow
        if (rxIndex >= sizeof(rxBuffer)) {
            rxIndex = 0;
        }

        // Trik Sederhana Modbus RTU Purwarupa:
        // Perintah 0x05 dan 0x06 Modbus panjangnya SELALU tepat 8 Byte!
        if (rxIndex >= 8) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(Sem_DataDiterima, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        // Pancing interupsi berikutnya
        HAL_UART_Receive_IT(&huart1, &rxChar, 1);
    }
}

// -----------------------------------------------------
// FUNC 0x01: READ COILS
// -----------------------------------------------------
void Function_Code_01_READ_COILS(uint8_t* rxBufferInput, uint8_t len) {
    // 1. Ambil Alamat Awal dan Jumlah Coil yang diminta HMI
    uint16_t start_addr = (rxBufferInput[2] << 8) | rxBufferInput[3];
    uint16_t quantity   = (rxBufferInput[4] << 8) | rxBufferInput[5];

    // 2. Validasi Batas Alamat (Gunakan MAX_COILS dari enum kita sebelumnya)
    if ((start_addr + quantity) <= MAX_COILS_READABLE) {

        uint8_t txBuffer[256];

        // 3. Hitung Jumlah Byte Data
        // Rumus membagi 8 dengan pembulatan ke atas (ceiling)
        uint8_t byte_count = (quantity + 7) / 8;

        // 4. Siapkan Header Balasan Modbus
        txBuffer[0] = rxBufferInput[0]; // Slave ID
        txBuffer[1] = 0x01;             // Function Code: 01 (Read Coils)
        txBuffer[2] = byte_count;       // Jumlah Byte Data

        // 5. Kosongkan (Nol-kan) area data sebelum diisi (Sangat Penting!)
        for (uint8_t i = 0; i < byte_count; i++) {
            txBuffer[3 + i] = 0x00;
        }

        // 6. Loop sebanyak kuantitas coil dan Lakukan "BIT PACKING"
        for (uint16_t i = 0; i < quantity; i++) {

            // Panggil fungsi Router yang sudah kita buat sebelumnya
            bool coil_status = Baca_Status_Coil(start_addr + i);

            if (coil_status == true) {
                // Tentukan lokasi byte dan bit ke berapa coil ini harus ditanam
                uint8_t byte_index = i / 8; // Contoh: i=9 -> masuk ke Byte indeks 1
                uint8_t bit_index  = i % 8; // Contoh: i=9 -> masuk ke Bit indeks 1

                // Gunakan operator Bitwise OR (|) untuk menghidupkan bit tertentu
                txBuffer[3 + byte_index] |= (1 << bit_index);
            }
        }

        // 7. Kalkulasi dan Pasang CRC16
        uint16_t crc_tx = Hitung_CRC16(txBuffer, 3 + byte_count);
        txBuffer[3 + byte_count] = crc_tx & 0xFF;         // CRC Low
        txBuffer[4 + byte_count] = (crc_tx >> 8) & 0xFF;  // CRC High

        // 8. Kirim Balasan ke HMI via RS485
        vTaskDelay(pdMS_TO_TICKS(5));
        HAL_UART_Transmit(&huart1, txBuffer, 5 + byte_count, 100);

    } else {
        // HMI meminta alamat di luar batas (Misal minta Coil ke-99)
        Send_Modbus_Exception(rxBufferInput[0], 0x01, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
    }
}

bool Baca_Status_Coil(uint16_t coil_addr) {
    switch (coil_addr) {
        case COIL_START_AUTO_WELD:
            return isAutoWeldActive;   // Variabel global dari FSM Anda

        case COIL_EMERGENCY_STOP:
            return isAborted;          // Variabel E-Stop

        case COIL_REQUEST_HOMING:
            return flagMintaHoming;

        case COIL_PAUSE_RESUME:
            return isPaused;

        case COIL_MANUAL_JOG:
            return flagManualJog;

        default:
            return false; // Alamat tidak valid
    }
}

// -----------------------------------------------------
// FUNC 0x02: READ DISCRETE INPUT
// -----------------------------------------------------
void Function_Code_02_READ_DISCRETE_INPUT(uint8_t* rxBufferInput, uint8_t len) {
    // 1. Ambil Alamat Awal dan Jumlah Input yang diminta HMI
    uint16_t start_addr = (rxBufferInput[2] << 8) | rxBufferInput[3];
    uint16_t quantity   = (rxBufferInput[4] << 8) | rxBufferInput[5];

    // 2. Validasi Batas Alamat
    if ((start_addr + quantity) <= MAX_DISCRETE_INPUTS) {

        uint8_t txBuffer[256];

        // 3. Hitung Jumlah Byte Data (Pembulatan ke atas)
        uint8_t byte_count = (quantity + 7) / 8;

        // 4. Siapkan Header Balasan Modbus
        txBuffer[0] = rxBufferInput[0]; // Slave ID
        txBuffer[1] = 0x02;             // Function Code: 02 (Read Discrete Inputs)
        txBuffer[2] = byte_count;       // Jumlah Byte Data

        // 5. Nol-kan area data
        for (uint8_t i = 0; i < byte_count; i++) {
            txBuffer[3 + i] = 0x00;
        }

        // 6. Loop Bit-Packing (Langsung baca dari pin Hardware!)
        for (uint16_t i = 0; i < quantity; i++) {

            // Panggil Router GPIO
            bool pin_status = Baca_Discrete_Input(start_addr + i);

            if (pin_status == true) {
                uint8_t byte_index = i / 8;
                uint8_t bit_index  = i % 8;

                txBuffer[3 + byte_index] |= (1 << bit_index);
            }
        }

        // 7. Kalkulasi dan Pasang CRC16
        uint16_t crc_tx = Hitung_CRC16(txBuffer, 3 + byte_count);
        txBuffer[3 + byte_count] = crc_tx & 0xFF;         // CRC Low
        txBuffer[4 + byte_count] = (crc_tx >> 8) & 0xFF;  // CRC High

        // 8. Kirim Balasan ke HMI
        vTaskDelay(pdMS_TO_TICKS(5));
        HAL_UART_Transmit(&huart1, txBuffer, 5 + byte_count, 100);

    } else {
        // HMI meminta alamat di luar batas Hardware yang ada
        Send_Modbus_Exception(rxBufferInput[0], 0x02, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
    }
}

bool Baca_Discrete_Input(uint16_t input_addr) {
    switch (input_addr) {

        case DI_LIMIT_SWITCH_X:
            // Asumsi Limit Switch menggunakan konfigurasi Active-Low (Ditekan = GND / RESET)
            return (HAL_GPIO_ReadPin(LIMIT_X_GPIO_Port, LIMIT_X_Pin) == LIMIT_X_Pressed);

        case DI_LIMIT_SWITCH_Y:
            return (HAL_GPIO_ReadPin(LIMIT_Y_GPIO_Port, LIMIT_Y_Pin) == LIMIT_Y_Pressed);

        case DI_LIMIT_SWITCH_Z:
            return (HAL_GPIO_ReadPin(LIMIT_Z_GPIO_Port, LIMIT_Z_Pin) == LIMIT_Z_Pressed);

        case DI_IS_MACHINE_HOMED:
            return isMachineHomed;

        default:
            return false; // Jika alamat tidak terdaftar
    }
}


// -----------------------------------------------------
// FUNC 0x03: READ HOLDING REGISTERS
// -----------------------------------------------------
void Function_Code_03_READ_HOLDING_REGISTER(uint8_t* rxBufferInput, uint8_t len) {
    uint16_t start_addr = (rxBufferInput[2] << 8) | rxBufferInput[3];
    uint16_t quantity   = (rxBufferInput[4] << 8) | rxBufferInput[5]; // Typo diperbaiki

    // Pengecekan batas aman memori Modbus
    if (start_addr + quantity <= TOTAL_HOLDING_REGISTERS) {

        // Modbus bisa meminta hingga 125 register (250 byte),
        // pastikan buffer cukup besar (maks standar Modbus RTU adalah 256 byte)
        uint8_t txBuffer[256];
        uint8_t byte_count = quantity * 2;

        txBuffer[0] = rxBufferInput[0]; // Slave ID
        txBuffer[1] = 0x03;             // Function Code: 03 (Read Holding Registers)
        txBuffer[2] = byte_count;       // Jumlah byte data yang akan dikirim

        // Ambil data dari Holding_Registers
        for (uint16_t i = 0; i < quantity; i++) {
            txBuffer[3 + (i * 2)] = (Holding_Registers[start_addr + i] >> 8) & 0xFF; // High Byte
            txBuffer[4 + (i * 2)] = Holding_Registers[start_addr + i] & 0xFF;        // Low Byte
        }

        // Kalkulasi CRC
        uint16_t crc_tx = Hitung_CRC16(txBuffer, 3 + byte_count);
        txBuffer[3 + byte_count] = crc_tx & 0xFF;         // CRC Low
        txBuffer[4 + byte_count] = (crc_tx >> 8) & 0xFF;  // CRC High

        // Jeda aman untuk Transceiver RS485
        vTaskDelay(pdMS_TO_TICKS(5));

        // Kirim balasan ke HMI/PC
        HAL_UART_Transmit(&huart1, txBuffer, 5 + byte_count, 100);

    } else {
        // Alamat melebihi batas, kirim Exception 0x02 (Illegal Data Address)
        Send_Modbus_Exception(rxBufferInput[0], 0x03, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
    }
}

// -----------------------------------------------------
// FUNC 0x04: READ INPUT REGISTERS (Monitor Posisi X & Y)
// -----------------------------------------------------
void Function_Code_04_READ_INPUT_REGISTER(uint8_t* rxBufferInput, uint8_t len) {
    uint16_t start_addr = (rxBufferInput[2] << 8) | rxBufferInput[3];
    uint16_t quantity   = (rxBufferInput[4] << 8) | rxBufferInput[5];

    // Asumsi TOTAL_INPUT_REGISTERS didefinisikan sebagai macro (misal: 10)
    if (start_addr + quantity <= TOTAL_INPUT_REGISTER) {
        uint8_t txBuffer[32];
        uint8_t byte_count = quantity * 2;

        txBuffer[0] = rxBufferInput[0]; // Slave ID
        txBuffer[1] = 0x04;
        txBuffer[2] = byte_count;

        for (uint16_t i = 0; i < quantity; i++) {
            txBuffer[3 + (i * 2)] = (Input_Registers[start_addr + i] >> 8) & 0xFF;
            txBuffer[4 + (i * 2)] = Input_Registers[start_addr + i] & 0xFF;
        }

        uint16_t crc_tx = Hitung_CRC16(txBuffer, 3 + byte_count);
        txBuffer[3 + byte_count] = crc_tx & 0xFF;
        txBuffer[4 + byte_count] = (crc_tx >> 8) & 0xFF;

        vTaskDelay(pdMS_TO_TICKS(5));
        HAL_UART_Transmit(&huart1, txBuffer, 5 + byte_count, 100);
    } else {
        // Alamat melebihi batas, kirim Exception 0x02 (Illegal Data Address)
        Send_Modbus_Exception(rxBufferInput[0], 0x04, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
    }
}

// -----------------------------------------------------
// FUNC 0x05: WRITE SINGLE COIL (Tombol Aksi HMI)
// -----------------------------------------------------
void Function_Code_05_WRITE_SINGLE_COIL(uint8_t* rxBufferInput, uint8_t len) {
    uint16_t coil_addr = (rxBufferInput[2] << 8) | rxBufferInput[3];
    uint16_t coil_val  = (rxBufferInput[4] << 8) | rxBufferInput[5];

    // Bendera penentu apakah STM32 akan mengirim balasan sukses (Echo)
    bool send_echo = true;

    switch (coil_addr) {
        case COIL_START_AUTO_WELD:
            if (coil_val == MODBUS_COIL_ON) {

                // --- 1. PRE-FLIGHT CHECK (SABUK PENGAMAN) ---
                if (configLas.totalKolom == 0 || configLas.totalBaris == 0 ||
                    configLas.pitchX == 0.0f  || configLas.pitchY == 0.0f) {

                    // Tolak eksekusi! Kirim Exception 0x03
                    Send_Modbus_Exception(rxBufferInput[0], 0x05, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    send_echo = false; // Batal kirim ACK
                    break;             // Langsung lompat keluar dari blok case ini
                }

                isAutoWeldActive = true;

            } else {
                // Opsional: Jika HMI mengirim nilai OFF (0x0000) untuk mematikan mode Auto
                isAutoWeldActive = false;
            }
            break;

        case COIL_EMERGENCY_STOP:
            if (coil_val == MODBUS_COIL_ON) {
                isAborted = true;
                isPaused = false;
            }
            break;

        case COIL_REQUEST_HOMING:
            if (coil_val == MODBUS_COIL_ON) {
                flagMintaHoming = true;
            }
            break;

        case COIL_PAUSE_RESUME:
            isPaused = (coil_val == MODBUS_COIL_ON);
            break;

        case COIL_MANUAL_JOG:
            if (coil_val == MODBUS_COIL_ON) {
                flagManualJog = true;
            }
            break;

        case COIL_SAVE_TO_FLASH:
            if (coil_val == MODBUS_COIL_ON) {
                if (!Save_Settings_To_Flash()) {
                    Send_Modbus_Exception(rxBufferInput[0], 0x05, MODBUS_EXCEPTION_BUSY);
                    send_echo = false;
                }
            }
            break;

        case COIL_RESET_SYSTEM:
            if (coil_val == MODBUS_COIL_ON) {
                Process_System_Reset(rxBufferInput, len);
                send_echo = false;
            }
            break;

        case COIL_CMD_ZERO_Z:
            if (coil_val == MODBUS_COIL_ON) {
                motorZ->setAsHome();
            }
            break;

        case COIL_CMD_UNLOCK_Z:
            // Cabut sabuk pengaman motor Z
            if (coil_val == MODBUS_COIL_ON) {
                motorZ->unlockPosition();
            }
            break;

        default:
            // HMI menembak alamat Coil yang tidak terdaftar
            Send_Modbus_Exception(rxBufferInput[0], 0x05, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
            send_echo = false;
            break;
    }

    // Hanya kirim Echo (ACK) jika semua proses aman dan tidak ada Exception
    if (send_echo) {
        vTaskDelay(pdMS_TO_TICKS(5));
        HAL_UART_Transmit(&huart1, rxBufferInput, len, 100);
    }
}
// -----------------------------------------------------
// FUNC 0x06: WRITE SINGLE REGISTER (Setting Mesin)
// -----------------------------------------------------
void Function_Code_06_WRITE_SINGLE_REGISTER(uint8_t* rxBufferInput, uint8_t len) {
    uint16_t alamat_offset = (rxBufferInput[2] << 8) | rxBufferInput[3];
    uint16_t data_baru     = (rxBufferInput[4] << 8) | rxBufferInput[5];

    // 1. Cek Batas Alamat Terlebih Dahulu
    if (alamat_offset >= TOTAL_HOLDING_REGISTERS) {
        Send_Modbus_Exception(rxBufferInput[0], 0x06, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
        return; // Keluar, alamat tidak masuk akal
    }

    // 2. Siapkan Bendera Validasi (Default: True)
    bool isDataValid = true;

    // 3. Validasi dan Konversi Data Sesuai Alamatnya
    switch (alamat_offset) {
        case 0: // Total Kolom
            if (data_baru == 0) {
                // Tidak mungkin ada las baterai dengan 0 kolom
                Send_Modbus_Exception(rxBufferInput[0], 0x06, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                isDataValid = false;
            } else {
                configLas.totalKolom = data_baru;
            }
            break;

        case 1: // Total Baris
            if (data_baru == 0) {
                // Tidak mungkin ada las baterai dengan 0 baris
                Send_Modbus_Exception(rxBufferInput[0], 0x06, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                isDataValid = false;
            } else {
                configLas.totalBaris = data_baru;
            }
            break;

        case 2: // Jarak Pitch X
            if (data_baru == 0) {
                Send_Modbus_Exception(rxBufferInput[0], 0x06, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                isDataValid = false; // Tandai bahwa data ini cacat!
            } else {
                configLas.pitchX = (float)data_baru / 10.0f;
            }
            break;

        case 3: // Jarak Pitch Y
            if (data_baru == 0) {
                Send_Modbus_Exception(rxBufferInput[0], 0x06, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                isDataValid = false; // Tandai cacat!
            } else {
                configLas.pitchY = (float)data_baru / 10.0f;
            }
            break;

        case 4:
            configLas.startKolom = (data_baru > 0) ? (data_baru - 1) : 0;
            break;

        case 5:
            configLas.startBaris = (data_baru > 0) ? (data_baru - 1) : 0;
            break;

        default:
            break; // Sisa register aman dan tidak perlu perlakuan khusus
    }

    // 4. JIKA DATA VALID: Baru Lakukan Penyimpanan dan Kirim ACK
    if (isDataValid == true) {

        // Simpan data mentah ke Holding Register (Aman, karena sudah terkurasi)
        Holding_Registers[alamat_offset] = data_baru;

        // Jika yang diubah adalah parameter mekanik, update objek motor
        if (alamat_offset >= 20) {
            Update_Motor_Param_From_Settings();
        }

        // Kirim Balasan Sukses (ACK)
        // Di FC06, balasan sukses adalah mengirim ulang pesan request (Echo)
        vTaskDelay(pdMS_TO_TICKS(5));
        HAL_UART_Transmit(&huart1, rxBufferInput, len, 100);
    }
}

// =========================================================
// FUNGSI MATEMATIKA CRC16 MODBUS
// =========================================================
uint16_t Hitung_CRC16(const uint8_t *nData, uint16_t wLength) {
    uint16_t wCRC = 0xFFFF;
    for (uint16_t i = 0; i < wLength; i++) {
        wCRC ^= (uint16_t)nData[i];
        for (uint16_t j = 8; j != 0; j--) {
            if ((wCRC & 0x0001) != 0) {
                wCRC >>= 1;
                wCRC ^= 0xA001; // Pastikan format heksadesimal valid
            } else {
                wCRC >>= 1;
            }
        }
    }
    return wCRC;
}

void Process_System_Reset(uint8_t* rxBufferInput, uint16_t len) {
    // 1. Echo ke HMI agar tidak Timeout
    HAL_UART_Transmit(&huart1, rxBufferInput, len, 100);

    // 2. Tunggu transmisi UART selesai
    vTaskDelay(pdMS_TO_TICKS(50));

    // 3. Reboot MCU
    NVIC_SystemReset();
}

void Send_Modbus_Exception(uint8_t slave_id, uint8_t function_code, uint8_t exception_code) {
    uint8_t txBuffer[5];
    txBuffer[0] = slave_id;
    txBuffer[1] = function_code | 0x80; // Standar Modbus: Function Code + 0x80 untuk menandakan Error
    txBuffer[2] = exception_code;       // Masukkan kode error (0x02 atau 0x06)

    uint16_t crc_tx = Hitung_CRC16(txBuffer, 3);
    txBuffer[3] = crc_tx & 0xFF;
    txBuffer[4] = (crc_tx >> 8) & 0xFF;

    vTaskDelay(pdMS_TO_TICKS(5));
    HAL_UART_Transmit(&huart1, txBuffer, 5, 100);
}

