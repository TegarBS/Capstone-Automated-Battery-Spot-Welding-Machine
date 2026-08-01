#include "Communication.h"
#include "GlobalData.h"

// =========================================================
// TASK KOMUNIKASI (PARSER MODBUS RTU)
// =========================================================
#include "Communication.h"

#include "FungsiEEPROM.h"
#include "GlobalData.h"
#include "UtilitasDanPerhitungan.h"

// =========================================================
// TASK KOMUNIKASI (PARSER MODBUS RTU ZERO-QUEUE)
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

            // -----------------------------------------------------
            // FUNC 0x04: READ INPUT REGISTERS (Monitor Posisi X & Y)
            // -----------------------------------------------------
            if (function_code == 0x04) {
                uint16_t start_addr = (rxBuffer[2] << 8) | rxBuffer[3];
                uint16_t quantity   = (rxBuffer[4] << 8) | rxBuffer[5];

                if (start_addr + quantity <= 10) {
                    uint8_t txBuffer[32];
                    uint8_t byte_count = quantity * 2;

                    txBuffer[0] = 0x01;
                    txBuffer[1] = 0x04;
                    txBuffer[2] = byte_count;

                    for (uint16_t i = 0; i < quantity; i++) {
                        txBuffer[3 + (i * 2)] = (Input_Registers[start_addr + i] >> 8) & 0xFF;
                        txBuffer[4 + (i * 2)] = Input_Registers[start_addr + i] & 0xFF;
                    }

                    uint16_t crc_tx = Hitung_CRC16(txBuffer, 3 + byte_count);
                    txBuffer[3 + byte_count] = crc_tx & 0xFF;
                    txBuffer[4 + byte_count] = (crc_tx >> 8) & 0xFF;

                    vTaskDelay(5);
                    HAL_UART_Transmit(&huart1, txBuffer, 5 + byte_count, 100);
                }
            }

            // -----------------------------------------------------
            // FUNC 0x05: WRITE SINGLE COIL (Tombol Aksi HMI)
            // -----------------------------------------------------
            else if (function_code == 0x05) {
                uint16_t coil_addr = (rxBuffer[2] << 8) | rxBuffer[3];
                uint16_t coil_val  = (rxBuffer[4] << 8) | rxBuffer[5];

                // COIL 0: START AUTO WELD
                if (coil_addr == 0x0000 && coil_val == 0xFF00) {
                    indexKolomAktual = 0; // Reset memori pola
                    indexBarisAktual = 0;
                    isAutoWeldActive = true; // Nyalakan mesin!
                }
                // COIL 1: ABORT / EMERGENCY STOP
                else if (coil_addr == 0x0001 && coil_val == 0xFF00) {
                    isAborted = true;
                    isPaused = false;
                }
                // COIL 2: REQUEST HOMING
                else if (coil_addr == 0x0002 && coil_val == 0xFF00) {
                    flagMintaHoming = true;
                }
                // COIL 3: PAUSE & RESUME
				else if (coil_addr == 0x0003) {
					if (coil_val == 0xFF00) isPaused = true;
					else isPaused = false;
				}
				// COIL 4: TRIGGER MODE MANUAL JOG
				else if (coil_addr == 0x0004 && coil_val == 0xFF00) {
					flagManualJog = true; // Bangunkan Sequencer untuk mode manual
				}
                // COIL 5: TRIGGER SAVE TO FLASH (EEPROM)
				else if (coil_addr == 0x0005 && coil_val == 0xFF00) {

					// Coba simpan ke Flash
					if (Save_Settings_To_Flash() == true) {
						// Jika berhasil (Mesin aman), Echo balik ke HMI tanda sukses
						HAL_UART_Transmit(&huart1, rxBuffer, len, 100);
					} else {
						// Jika Gagal (Mesin sibuk), Kirim Modbus Exception 0x06 (Slave Device Busy)
						uint8_t txBuffer[5];
						txBuffer[0] = rxBuffer[0];          // Slave ID
						txBuffer[1] = function_code | 0x80; // Function Code + 0x80 (Penanda Error)
						txBuffer[2] = 0x06;                 // Exception 0x06: Slave Device Busy

						uint16_t crc_tx = Hitung_CRC16(txBuffer, 3);
						txBuffer[3] = crc_tx & 0xFF;
						txBuffer[4] = (crc_tx >> 8) & 0xFF;

						HAL_UART_Transmit(&huart1, txBuffer, 5, 100);
					}
				}

                // -----------------------------------------------------
				// COIL 6: RESET SYSTEM (REBOOT MCU)
				// -----------------------------------------------------
				else if (coil_addr == 0x0006 && coil_val == 0xFF00) {

					// 1. ECHO (BALAS) KE HMI TERLEBIH DAHULU!
					// Ini wajib agar HMI Python menerima jawaban "OK" dan tidak terkena Timeout Error.
					HAL_UART_Transmit(&huart1, rxBuffer, len, 100);

					// 2. Beri waktu sebentar (50ms) agar sinyal UART benar-benar selesai terkirim dari kabel
					vTaskDelay(pdMS_TO_TICKS(50));

					// 3. TARIK PELATUK RESET!
					// Perintah ini akan langsung mematikan dan menghidupkan ulang seluruh sistem STM32
					NVIC_SystemReset();
				}

                // Echo balik ke Master (HMI) tanda sukses
                vTaskDelay(5);
                HAL_UART_Transmit(&huart1, rxBuffer, len, 100);
            }

            // -----------------------------------------------------
            // FUNC 0x06: WRITE SINGLE REGISTER (Setting Mesin)
            // -----------------------------------------------------
            else if (function_code == 0x06) {
                uint16_t alamat_offset = (rxBuffer[2] << 8) | rxBuffer[3];
                uint16_t data_baru     = (rxBuffer[4] << 8) | rxBuffer[5];

                if (alamat_offset < TOTAL_HOLDING_REGISTERS) {
                    Holding_Registers[alamat_offset] = data_baru;

                    // MENGISI "OTAK" MESIN (Sinkronisasi ke configLas)
                    // HMI harus mengirim data pitch dikali 10 (contoh: 18.5mm jadi 185)
                    if (alamat_offset == 0) configLas.totalKolom = data_baru;
                    if (alamat_offset == 1) configLas.totalBaris = data_baru;
                    if (alamat_offset == 2) configLas.pitchX = (float)data_baru / 10.0f;
                    if (alamat_offset == 3) configLas.pitchY = (float)data_baru / 10.0f;

                    // Catatan: Reg 4 (Drop Delay) dan Reg 5 (Weld Time) biarkan
                    // tersimpan di Holding_Registers karena dibaca langsung oleh Sequencer
                }

                if (alamat_offset >= 20) {
					// Panggil fungsinya di sini! Mesin akan langsung ngebut/melambat
					// sesuai settingan baru detik itu juga tanpa perlu disuruh Homing.
					Update_Motor_Speed_From_Settings();
				}

                vTaskDelay(5);
                HAL_UART_Transmit(&huart1, rxBuffer, len, 100);
            }

            // -----------------------------------------------------
            // ILLEGAL FUNCTION / EXCEPTION
            // -----------------------------------------------------
            else {
                uint8_t txBuffer[5];
                txBuffer[0] = rxBuffer[0];
                txBuffer[1] = function_code | 0x80;
                txBuffer[2] = 0x01;

                uint16_t crc_tx = Hitung_CRC16(txBuffer, 3);
                txBuffer[3] = crc_tx & 0xFF;
                txBuffer[4] = (crc_tx >> 8) & 0xFF;

                vTaskDelay(5);
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
