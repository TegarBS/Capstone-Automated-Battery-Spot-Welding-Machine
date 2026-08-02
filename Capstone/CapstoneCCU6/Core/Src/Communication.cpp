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
const uint8_t  MODBUS_EXCEPTION_ILLEGAL_ADDRESS  = 0x02; // Alamat tidak ditemukan
const uint8_t  MODBUS_EXCEPTION_BUSY             = 0x06; // Perangkat sibuk
const uint8_t  MODBUS_EXCEPTION_ILLEGAL_FUNCTION = 0x01; // Illegal Function

// --- Daftar Alamat Coil ---
enum ModbusCoilAddress {
    COIL_START_AUTO_WELD = 0x0000,
    COIL_EMERGENCY_STOP  = 0x0001,
    COIL_REQUEST_HOMING  = 0x0002,
    COIL_PAUSE_RESUME    = 0x0003,
    COIL_MANUAL_JOG      = 0x0004,
    COIL_SAVE_TO_FLASH   = 0x0005,
    COIL_RESET_SYSTEM    = 0x0006
};

// =========================================================
// TASK KOMUNIKASI (PARSER MODBUS RTU ZERO-QUEUE)
// =========================================================
void Task_Komunikasi(void *pvParameters) {
    // Definisi konstanta ID Slave agar tidak menggunakan magic number (0x01)
    const uint8_t SLAVE_ID = 0x01;

    while(1) {
        if (xSemaphoreTake(Sem_DataDiterima, portMAX_DELAY) == pdTRUE) {

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
                case 0x04:
                	Function_Code_04(rxBuffer, len);
                    break;

                case 0x05:
                	Function_Code_05(rxBuffer, len);
                    break;

                case 0x06:
                	Function_Code_06(rxBuffer, len);
                    break;

                default:
                    Send_Modbus_Exception(rxBuffer[0], function_code, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
                    break;
            }
        }
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
// FUNC 0x04: READ INPUT REGISTERS (Monitor Posisi X & Y)
// -----------------------------------------------------
void Function_Code_04(uint8_t* rxBufferInput, uint8_t len) {
    uint16_t start_addr = (rxBufferInput[2] << 8) | rxBufferInput[3];
    uint16_t quantity   = (rxBufferInput[4] << 8) | rxBuffer[5];

    // Asumsi TOTAL_INPUT_REGISTERS didefinisikan sebagai macro (misal: 10)
    if (start_addr + quantity <= 10) {
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
void Function_Code_05(uint8_t* rxBufferInput, uint8_t len) {
    uint16_t coil_addr = (rxBufferInput[2] << 8) | rxBufferInput[3];
    uint16_t coil_val  = (rxBufferInput[4] << 8) | rxBufferInput[5];

    bool send_echo = true;

    switch (coil_addr) {
        case COIL_START_AUTO_WELD:
            if (coil_val == MODBUS_COIL_ON) {
                indexKolomAktual = 0;
                indexBarisAktual = 0;
                isAutoWeldActive = true;
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

        default:
            Send_Modbus_Exception(rxBufferInput[0], 0x05, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
            send_echo = false;
            break;
    }

    if (send_echo) {
        vTaskDelay(pdMS_TO_TICKS(5));
        HAL_UART_Transmit(&huart1, rxBufferInput, len, 100);
    }
}

// -----------------------------------------------------
// FUNC 0x06: WRITE SINGLE REGISTER (Setting Mesin)
// -----------------------------------------------------
void Function_Code_06(uint8_t* rxBufferInput, uint8_t len) {
    uint16_t alamat_offset = (rxBufferInput[2] << 8) | rxBufferInput[3];
    uint16_t data_baru     = (rxBufferInput[4] << 8) | rxBufferInput[5];

    if (alamat_offset < TOTAL_HOLDING_REGISTERS) {
        Holding_Registers[alamat_offset] = data_baru;

        // Gunakan switch-case agar sinkronisasi lebih rapi
        switch (alamat_offset) {
            case 0: configLas.totalKolom = data_baru; break;
            case 1: configLas.totalBaris = data_baru; break;
            case 2: configLas.pitchX = (float)data_baru / 10.0f; break;
            case 3: configLas.pitchY = (float)data_baru / 10.0f; break;
            default: break;
        }

        if (alamat_offset >= 20) {
            Update_Motor_Speed_From_Settings();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
        HAL_UART_Transmit(&huart1, rxBufferInput, len, 100);
    } else {
        // Alamat melebihi batas
        Send_Modbus_Exception(rxBufferInput[0], 0x06, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
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
