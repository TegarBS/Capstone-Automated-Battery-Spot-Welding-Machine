#include "Communication.h"
#include "GlobalData.h"

// =========================================================
// TASK KOMUNIKASI (PARSER MODBUS RTU)
// =========================================================
void Task_Komunikasi(void *pvParameters) {
    while(1) {
        if (xSemaphoreTake(Sem_DataDiterima, portMAX_DELAY) == pdTRUE) {

            uint8_t len = rxIndex;
            rxIndex = 0; // Reset untuk paket berikutnya

            // 1. Validasi Slave ID (ID Mesin kita = 0x01)
            if (rxBuffer[0] != 0x01) continue;

            // 2. Validasi CRC16 (Menangkal Noise)
            uint16_t crc_hitung = Hitung_CRC16(rxBuffer, len - 2);
            uint16_t crc_diterima = (rxBuffer[len - 1] << 8) | rxBuffer[len - 2];
            if (crc_hitung != crc_diterima) continue;

            uint8_t function_code = rxBuffer[1];

            if (function_code == 0x04) {
				uint16_t start_addr = (rxBuffer[2] << 8) | rxBuffer[3];
				uint16_t quantity   = (rxBuffer[4] << 8) | rxBuffer[5];

				// Validasi agar HMI tidak membaca di luar batas array (maks 10)
				if (start_addr + quantity <= 10) {
					uint8_t txBuffer[32];
					uint8_t byte_count = quantity * 2;

					txBuffer[0] = 0x01;         // Slave ID
					txBuffer[1] = 0x04;         // Function Code
					txBuffer[2] = byte_count;   // Jumlah Byte Data

					// Masukkan data 16-bit ke dalam frame 8-bit (High Byte lalu Low Byte)
					for (uint16_t i = 0; i < quantity; i++) {
						txBuffer[3 + (i * 2)] = (Input_Registers[start_addr + i] >> 8) & 0xFF;
						txBuffer[4 + (i * 2)] = Input_Registers[start_addr + i] & 0xFF;
					}

					// Hitung CRC16 untuk paket balasan
					uint16_t crc_tx = Hitung_CRC16(txBuffer, 3 + byte_count);
					txBuffer[3 + byte_count] = crc_tx & 0xFF;        // CRC Low Byte
					txBuffer[4 + byte_count] = (crc_tx >> 8) & 0xFF; // CRC High Byte

					// Kirim balasan ke HMI
					HAL_UART_Transmit(&huart1, txBuffer, 5 + byte_count, 100);
				}
			}

            else if (function_code == 0x05) {
                uint16_t coil_addr = (rxBuffer[2] << 8) | rxBuffer[3];
                uint16_t coil_val  = (rxBuffer[4] << 8) | rxBuffer[5];

                // Jika HMI kirim Coil 00001 (0x0000) bernilai ON (0xFF00)
                if (coil_addr == 0x0000 && coil_val == 0xFF00) {
                    Koordinat_t titikBaru;

                    // Rangkai X (Reg 0 & 1) dan Y (Reg 2 & 3) dari Holding Registers
                    titikBaru.x = ((long)Holding_Registers[1] << 16) | Holding_Registers[0];
                    titikBaru.y = ((long)Holding_Registers[3] << 16) | Holding_Registers[2];

                    // Masukkan titik utuh tersebut ke Queue
                    xQueueSend(Queue_Koordinat, &titikBaru, 0);
                }else if (coil_addr == 0x0001 && coil_val == 0xFF00) {
                    isAborted = true; // Aktifkan bendera rem darurat!
                    isPaused = false; // Batalkan jeda jika sedang di-jeda
                }else if (coil_addr == 0x0002 && coil_val == 0xFF00) {
                	flagHoming = true;
                    Koordinat_t cmdHome = {-999999, -999999}; // Kode khusus Homing
                    xQueueSend(Queue_Koordinat, &cmdHome, 0);
                }else if (coil_addr == 0x0003) {
                    if (coil_val == 0xFF00) isPaused = true;  // ON = Jeda
                    else isPaused = false;                    // OFF = Lanjut
                }

                // Echo balik ke Master (HMI)
                HAL_UART_Transmit(&huart1, rxBuffer, len, 100);
            }

            else if (function_code == 0x06) {
				uint16_t alamat_offset = (rxBuffer[2] << 8) | rxBuffer[3];
				uint16_t data_baru     = (rxBuffer[4] << 8) | rxBuffer[5];

				if (alamat_offset < 10) {
					Holding_Registers[alamat_offset] = data_baru;
				}

				// Echo balik ke Master (HMI) sesuai standar Modbus
				HAL_UART_Transmit(&huart1, rxBuffer, len, 100);
			}

            else {
				uint8_t txBuffer[5]; // Ukuran paket Exception selalu 5 Byte

				txBuffer[0] = rxBuffer[0];          // Slave ID (0x01)
				txBuffer[1] = function_code | 0x80; // Function Code ditambah 0x80 (Penanda Error)
				txBuffer[2] = 0x01;                 // Exception Code: 0x01 (Illegal Function)

				// Hitung CRC16 untuk 3 byte pertama
				uint16_t crc_tx = Hitung_CRC16(txBuffer, 3);
				txBuffer[3] = crc_tx & 0xFF;        // CRC Low Byte
				txBuffer[4] = (crc_tx >> 8) & 0xFF; // CRC High Byte

				// Tembakkan balasan error ke HMI
				HAL_UART_Transmit(&huart1, txBuffer, 5, 100);
			}


        }
    }
}

// =========================================================
// CALLBACK INTERUPSI UART (HARDWARE ISR)
// =========================================================
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

// =========================================================
// FUNGSI MATEMATIKA CRC16 MODBUS
// =========================================================
uint16_t Hitung_CRC16(uint8_t *nData, uint16_t wLength) {
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
