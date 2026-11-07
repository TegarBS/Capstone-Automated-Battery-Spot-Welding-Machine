# 🔋 Automated Battery Spot Welding Machine

![STM32](https://img.shields.io/badge/MCU-STM32F401-03234B?logo=stmicroelectronics&logoColor=white)
![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)
![C++](https://img.shields.io/badge/Language-C%2FC%2B%2B-00599C?logo=cplusplus&logoColor=white)
![Modbus](https://img.shields.io/badge/Protocol-Modbus%20RTU%20(RS--485)-orange)
![Status](https://img.shields.io/badge/Testing-in%20progress-yellow)

A moving-bed, 3-axis automated spot welding machine for assembling lithium-ion battery packs (18650 / 21700 cylindrical cells with pure nickel strip). An **STM32F401** running **FreeRTOS** controls the motion, pneumatic electrode head and weld trigger, while the operator configures the weld matrix from a **Haiwell HMI** over **Modbus RTU (RS-485)**.

This is a capstone project from the S1 Electrical Engineering program, Universitas Sebelas Maret (2026), covering mechanical design, PCB design, embedded firmware, pneumatics and machine safety in one product.

<img width="1920" height="1080" alt="Machine overview" src="https://github.com/user-attachments/assets/38f92ba0-b8f0-4d25-97ee-73e730a2df13" />

---

## 🎯 Problem & Motivation

Hand-held spot welding pens are common for building battery packs, but they have clear weaknesses:

- **Inconsistent weld quality:** electrode pressure varies between points and operators, so contact resistance varies. Too little pressure causes sparks and can burn through the cell casing.
- **Low speed and poor ergonomics:** packs of 100–200 cells cause fatigue and misplaced weld points.
- **High current and EMI:** weld pulses reach hundreds to thousands of amperes and can disturb microcontrollers and communication lines.

This machine addresses them with programmed weld positions, a spring-loaded pneumatic electrode head for uniform pressure, and layered hardware and firmware safety.

---

## ✨ Key Features

- **Moving-bed architecture:** the bed (Y axis) moves while the X bridge stays fixed. This needs only one center-drive lead screw for Y and avoids racking.
- **Configurable weld matrix:** columns, rows, pitch X/Y, start point and dwell time are set from the HMI.
- **Zero-array on-the-fly path generation:** weld coordinates are computed from HMI parameters, so no point array is stored in memory.
- **Job resume:** the operator can choose the start column/row and continue from there instead of restarting at point 0.
- **Hybrid Z axis:** a NEMA 17 + T8 lead screw sets base height, and a pneumatic cylinder performs the weld press.
- **Spring-loaded floating weld head:** each electrode needle has its own spring, which absorbs height variation between cells.
- **Layered safety interlock:** the weld pulse is only allowed when the Z-Down reed switch is active, the emergency stop is released and the watchdog is healthy. If the MCU freezes, hardware logic holds the weld trigger line off.
- **Galvanic isolation:** optocouplers (6N137 / PC817) separate the 3.3 V logic from 5 V / 24 V and weld-power circuits. Separate power rails limit EMI.
- **Configuration persistence:** settings are stored in an external I2C EEPROM.
- **HMI diagnostics:** error codes shown on a dedicated Panel Error page (see [Error Codes](#-hmi-error-codes)).

---

## 🧱 System Architecture

| Subsystem | Description |
|---|---|
| **Power distribution & protection** | 220 VAC → MCB → latching emergency button → separate rails: 24 V PSU (motor drivers, solenoid valves, indicators), 24→5 V buck converter (interface and sensor logic), isolated 3.3 V supply for the MCU. |
| **Central controller** | STM32F401 (ARM Cortex-M4) runs FreeRTOS, generates step pulses with hardware timers, sequences the weld cycle with an FSM and handles safety interrupts and serial communication. |
| **Operator console** | Haiwell B7H 7" HMI (Modbus RTU master) for parameters, manual control and alarms. |
| **XYZ motion** | STEP/DIR through high-speed optocouplers to microstepping drivers. NEMA 23 on X and Y, NEMA 17 on Z. |
| **Pneumatics & welding** | Compressor → filter-regulator → solenoid valves (press down / up) → double-acting cylinder. Two reed switches report Z-Down / Z-Up. The weld trigger goes through an optocoupler to the Glitter 801B welder (MT / pedal-trigger mode). |

### Firmware (STM32 / C++)

- Built with **STM32CubeIDE**, **HAL** and **FreeRTOS (CMSIS-RTOS)**.
- FreeRTOS tasks: motion (pulse generation and FSM), communication (Modbus), monitoring, and EEPROM storage.
- Modular object-oriented design (e.g. `StepperMotor` class, pneumatic control, timing control).
- **Modbus RTU slave** (holding/input registers and coils) with CRC-16 validation.
- **IWDG** hardware watchdog.
- Follows **MISRA C**-style practices: static allocation (no `malloc`/`new` at runtime), single entry/single exit, and divide-by-zero protection in pulse-to-mm conversion.
- Static analysis with **Cppcheck**.

**Weld cycle (FSM):**

```text
Load config (EEPROM + HMI) → Homing XYZ → Move to next point
→ Cylinder down → Z-Down reed active? (timeout → abort)
→ Weld trigger → Cylinder up → Z-Up reed active? (timeout → abort)
→ Last point? No → next point / Yes → done
```

**Point coordinates:**

```text
X(i) = X0 + i · PitchX      i = 0 … (columns − 1)
Y(j) = Y0 + j · PitchY      j = 0 … (rows − 1)
```

Distances are converted to pulses with `steps/mm = (200 × microstep) / lead (mm/rev)`, with optional backlash compensation on direction reversal.

### HMI (Haiwell B7H, designed in Haiwell Cloud SCADA)

- **Dashboard:** Start Auto Weld, Pause, Stop, Home; battery settings (columns, rows, pitch X/Y); axis position.
- **Manual:** per-axis manual input, Z up/down, manual weld, actual position, limit and navigation status.
- **Panel Error:** active alarm and error codes.
- **Calibration:** motion parameters (steps, max/min speed, accel, max travel, invert, homing) and weld parameters (spot spacing, offsets, weld count, pneumatic timeout, weld pulse).

<img width="769" height="454" alt="image" src="https://github.com/user-attachments/assets/be9b0b71-8b75-48d5-a255-07fc1068fb90" />


---

## 🔧 Hardware Specification

| Parameter | Specification |
|---|---|
| Architecture | Moving bed (Y moves, X bridge static) |
| Bed size / max weld area | 400 × 400 mm / 350 × 350 mm |
| Y travel | Up to 350 mm |
| Design load | Up to 30 kg |
| Microcontroller | STM32F401CCU6 (Black Pill) |
| X / Y drive | NEMA 23, microstepping driver; T8 (X) and T12 (Y) lead screw; HGR15 linear rails |
| Z drive | NEMA 17 + T8 lead screw (base height) + MAL 16×25 pneumatic cylinder (weld press) |
| Working pressure | Fixed at 5 bar (theoretical cylinder force ≈ 100 N) |
| Weld unit | Glitter 801B capacitor-discharge welder with 73SA spring-loaded head, MT mode |
| HMI | Haiwell B7H 7", Modbus RTU over RS-485 |
| Stepper drivers | DM542 (×2), TB6600 (×1) |
| Safety | Reed switches (Z-Down / Z-Up), optocouplers, IWDG, emergency stop |
| Custom PCB | 2-layer "CNC Auto Spot Welder" V2.0 (EasyEDA) |
| Mechanical | Aluminum 4040 profile frame, C-Beam X axis, Bakelite bed and fixture, 3D-printed (PLA) brackets, cable drag chain |

**Key design decisions**

| Decision | Chosen | Why |
|---|---|---|
| Kinematics | Moving bed | One center drive for Y, about 35% lower Y-drive cost than a moving gantry, simpler assembly |
| Transmission | Lead screw T8/T12 | Low cost, easy to source, backlash compensated in firmware (vs. GT2 belt stretch or expensive ball screw) |
| Weld unit placement | Static on X bridge | Keeps the X carriage light (< 2.5 kg), avoids vibration affecting the welder's capacitors, and keeps weld cables short (40–50 cm) |

---

## 📡 Modbus RTU Interface

The HMI is the **master** and the STM32 is the **slave** over RS-485 (via an RS-485-to-TTL transceiver module). The HMI writes the matrix parameters (rows, columns, pitch X/Y, dwell time) into STM32 registers.

> Serial settings: baud rate `19200`, parity `none`, slave ID `01`.

---

## ⚠️ HMI Error Codes

| Code | Meaning |
|---|---|
| 101 | EEPROM / storage error (failed to read or write parameters) |
| 102 | I2C communication error to EEPROM IC (cable disconnected) |
| 103 | Failed to save config to EEPROM (config used without saving) |
| 104 / 105 | I2C error on load / save of ODO (odometer) |
| 201 / 202 / 203 | Hard limit hit on X / Y / Z |
| 206 / 207 / 208 | Out of bounds / soft limit on X / Y / Z (Modbus command exceeds physical travel) |
| 301 | Pneumatic Z-Up timeout (upper sensor not triggered in time) |
| 302 | Pneumatic Z-Down timeout (lower sensor not triggered in time) |
| 303 | Weld misfire (welder failed to fire or Timer 5 timeout) |
| 403 | Aborted |
| 501 | Start column/row index exceeds total columns (auto-reset to 1) |
| 502 | ABORT: battery columns/rows exceed max X/Y travel |

---

## 🧪 Testing & Performance

> 🚧 **Testing is in progress.** Results will be added as each test is completed.

Seven test aspects were agreed with the project supervisor. The table lists the pass criteria; results are filled in as testing finishes.

| # | Aspect | Method | Pass criterion | Result |
|---|---|---|---|---|
| 1 | X/Y position accuracy | Command a position, measure with digital caliper, repeat ≥ 10× | Mean error ≤ ±0.3 mm | ⏳ In progress |
| 2 | Repeatability & backlash | Approach same points from both directions, with and without firmware compensation | Repeatability ≤ ±0.2 mm | ⏳ In progress |
| 3 | Weld quality | Test matrix at 5 bar; visual inspection and pull/peel test | ≥ 95% of points without burn-through or expulsion | ⏳ In progress |
| 4 | Safety interlock | Z-Down inactive, E-stop pressed, MCU off, watchdog timeout | No weld pulse in any failure scenario | ⏳ In progress |
| 5 | EMI immunity | Repeated welding while polling Modbus and monitoring resets | Zero spontaneous resets, CRC errors ≈ 0 | ⏳ In progress |
| 6 | HMI communication | Write 5 parameters from the HMI and read them back | ≥ 99% successful transactions | ⏳ In progress |
| 7 | Spot-weld resistance | Compare weld resistance with a reference nickel strip (internal resistance meter) | Close to reference, not significantly higher | ⏳ In progress |

> 📌 *Add photos of test setups and weld results here (`docs/images/test-*.jpg`).*

---

## 📂 Repository Structure

```text
├── Capstone/      # STM32CubeIDE project (Core, Drivers, FreeRTOS tasks, Modbus slave parser)
├── README.md
└── LICENSE
```

> 💡 Suggested additions: `Hardware/` (schematic, PCB Gerber, CAD), `HMI/` (Haiwell project file) and `docs/` (images, report).

---

## 🛠️ Build & Flash

1. Install **STM32CubeIDE**.
2. Clone the repository:
   ```bash
   git clone https://github.com/TegarBS/Capstone-Automated-Battery-Spot-Welding-Machine.git
   ```
3. In STM32CubeIDE: `File → Import → Existing Projects into Workspace` and select the `Capstone/` folder.
4. Build the project and flash the STM32F401 (Black Pill) with an **ST-LINK V2** (or STM32CubeProgrammer).
5. Load the HMI project into the Haiwell B7H using Haiwell Cloud SCADA.
6. Connect the HMI to the STM32 over RS-485 and set the matching serial parameters on both sides.

---

## 🧰 Tools Used

| Purpose | Tool |
|---|---|
| Mechanical CAD | Autodesk Fusion |
| Schematic & PCB | EasyEDA |
| Firmware | STM32CubeIDE, HAL, FreeRTOS (CMSIS-RTOS), STM32CubeProgrammer |
| HMI | Haiwell Cloud SCADA |
| Communication testing | Modbus Poll |
| Static analysis | Cppcheck |

---

## 📸 Gallery

> 📌 *Add: front view, close-up of the Z head and bed, control box, PCB V2.0, CAD renders, welded nickel strips on cells.*

---

## 👥 Team

| Name | Role |
|---|---|
| **Emir Dzacky Marcello** | HMI design, schematic design, wiring and chassis assembly |
| **Tegar Bayu Segara** | Firmware development, mechanical CAD design, PCB design, control panel assembly |

Program Studi S1 Teknik Elektro, Fakultas Teknik, Universitas Sebelas Maret, Surakarta.

---

## ⚖️ Scope & Limitations

- The project controls the weld **trigger** only. It does not design the weld power circuit or analyze the effect of current or pulse duration on weld quality.
- Tested with cylindrical Li-ion cells and pure nickel strip only.
- All tests run at a fixed 5 bar cylinder pressure and a fixed welder energy level.

---

## 📄 License

See [LICENSE](LICENSE).
