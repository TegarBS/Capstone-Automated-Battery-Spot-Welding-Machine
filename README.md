# 🔋 Automated Battery Spot Welding Machine

An industrial-grade, precision-controlled automated spot welding machine designed for battery pack assembly. This project implements a robust embedded architecture utilizing an **STM32** microcontroller communicating directly with an industrial **Haiwell HMI** via **Modbus RTU**[cite: 3, 6].

---

## 🚀 System Architecture

- **Firmware (STM32 / C++):** 
  * Built using **STM32CubeIDE** and **FreeRTOS** for real-time multi-tasking (motion control, sequencing, and communication)[cite: 3].
  * Modular object-oriented design for stepper motors (`StepperMotor` class), pneumatic systems, and precise timing control[cite: 3].
  * Implements industrial **Modbus RTU** communication protocol (Handling Holding/Input Registers and Coils) and independent hardware watchdog monitoring (**IWDG**) for system safety[cite: 3, 8].
- **HMI (Haiwell Industrial Panel / Modbus RTU):**
  * Industrial Human-Machine Interface connected directly via RS-485 (Modbus RTU) to monitor welding parameters, manage coordinates, handle system alarms, and track production status in real-time[cite: 3, 6, 8].

---

## 📂 Repository Structure

```text
├── Capstone/      # STM32CubeIDE C++ project files (Core, Drivers, FreeRTOS tasks, Modbus Slave Parser)[cite: 3]
├── README.md
├── LICENSE
