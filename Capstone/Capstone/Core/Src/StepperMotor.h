/*
 * StepperMotor.h
 *
 *  Created on: Jul 13, 2026
 *      Author: Tegar
 */

#ifndef SRC_STEPPERMOTOR_H_
#define SRC_STEPPERMOTOR_H_

#include "main.h"
#include <stdlib.h> // Diperlukan untuk fungsi abs() jika dibutuhkan nanti

class StepperMotor {
private:
    // Pointer ke alamat register Port (misal: GPIOB) dan nomor Pin
    GPIO_TypeDef* stepPort;
    uint16_t stepPin;
    GPIO_TypeDef* dirPort;
    uint16_t dirPin;

    // Variabel pelacak koordinat
    long currentPosition;
    long targetPosition;

public:
    // Bendera status pergerakan motor
    bool isMoving(void);

    // Constructor: Dipanggil saat objek motor pertama kali dibuat
    StepperMotor(GPIO_TypeDef* step_port, uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin);

    // Fungsi untuk menetapkan target koordinat baru
    void setTargetPulse(long target);

    // Fungsi utama yang akan dipanggil oleh Timer Interrupt setiap sekian mikrodetik
    void update();

    // Fungsi kontrol darurat dan kalibrasi
    void stop();
    void resetPosition();
    long getCurrentPosition();
};



#endif /* SRC_STEPPERMOTOR_H_ */
