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

    long currentPositionPulse;
    long targetPositionPulse;
    bool invertDir;

    uint32_t timerClock; // Kecepatan clock timer

    // --- Variabel Dynamic ARR (Akselerasi Trapesium) ---
    MotionState state;
    long stepsToGo;
    long accelSteps;

    // --- Variabel Pergerakan ---
    float minSpeedHz;       // Frekuensi awal saat motor mulai bergerak
    float maxSpeedHz;       // Frekuensi puncak / cruising
    float currentSpeedHz;   // Frekuensi aktual saat ini
    float deltaSpeedHz;    	// Lonjakan frekuensi per langkah
    
    float accelHz;


public:
    bool isMoving;

    // Constructor kini meminta input Hardware Timer
    StepperMotor(GPIO_TypeDef* step_port, uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin, TIM_HandleTypeDef* timer);

    void setTarget(long target);
    void setInvertDirection(bool invert);

	void setMinSpeedHz(float minSpeedHz);
    void setMaxSpeedHz(float maxSpeedHz);
	void setAccelHz(float accelHz);

    // Fungsi ini BUKAN dipanggil dari loop, tapi langsung oleh Interupsi Timer!
    void isr_update(void);

    void stop(void);
    void resetPosition(void);
    long getCurrentPositionPulse(void);
};

#endif /* SRC_STEPPERMOTOR_H_ */
