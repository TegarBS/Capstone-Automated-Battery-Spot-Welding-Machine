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

    uint32_t currentARR;    // Nilai ARR saat ini yang disuntikkan ke Timer
    uint32_t minARR;        // ARR terkecil = Kecepatan Puncak (20.833 Hz)
    uint32_t startARR;      // ARR terbesar = Kecepatan Awal/Berhenti (2.000 Hz)
    uint32_t arrDelta;      // Nilai perubahan ARR setiap step

public:
    bool isMoving;

    // Constructor kini meminta input Hardware Timer
    StepperMotor(GPIO_TypeDef* step_port, uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin, TIM_HandleTypeDef* timer);

    void setTarget(long target);
    void setInvertDirection(bool invert);

    // Fungsi ini BUKAN dipanggil dari loop, tapi langsung oleh Interupsi Timer!
    void isr_update();

    void stop();
    void resetPosition();
    long getCurrentPosition();
};

#endif /* SRC_STEPPERMOTOR_H_ */
