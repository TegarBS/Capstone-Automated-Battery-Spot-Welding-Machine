#ifndef SRC_STEPPERMOTOR_H_
#define SRC_STEPPERMOTOR_H_

#include "main.h"
#include <stdlib.h> // Untuk abs()

enum MotionState {
    IDLE_STATE,
    ACCELERATING,
    CRUISING,
    DECELERATING
};

class StepperMotor {
private:
    GPIO_TypeDef* stepPort;
    uint16_t stepPin;
    GPIO_TypeDef* dirPort;
    uint16_t dirPin;
    TIM_HandleTypeDef* htim; // Pointer ke Hardware Timer (TIM2 / TIM3)

    long currentPosition;
    long targetPosition;
    bool invertDir;

    // --- Variabel Dynamic ARR (Akselerasi Trapesium) ---
    MotionState state;
    long stepsToGo;
    long accelSteps;

    // Hapus variabel minARR, startARR, currentARR, arrDelta
    // Ganti menjadi:
    float minFreq;       // Frekuensi awal saat motor mulai bergerak (misal: 500 Hz)
    float maxFreq;       // Frekuensi puncak / cruising (misal: 20000 Hz)
    float currentFreq;   // Frekuensi aktual saat ini
    float freqDelta;     // Lonjakan frekuensi per langkah
    uint32_t timerClock; // Kecepatan clock timer (misal: 4000000)

public:
    bool isMoving;

    // Constructor kini meminta input Hardware Timer
    StepperMotor(GPIO_TypeDef* step_port, uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin, TIM_HandleTypeDef* timer, float MIN_SPEED);

    void setTarget(long target);
    void setInvertDirection(bool invert);
    void setMaxSpeed(float newMaxFreq);

    // Fungsi ini BUKAN dipanggil dari loop, tapi langsung oleh Interupsi Timer!
    void isr_update();

    void stop();
    void resetPosition();
    long getCurrentPosition();
};

#endif /* SRC_STEPPERMOTOR_H_ */
