# PID-Temperature-Control
C programmed MSP430 interface with Matlab GUI
# PID Temperature Controller (MSP430 + MATLAB GUI)

A fully integrated PID-controlled thermoelectric oven/cooler system, developed using an MSP430G2553 microcontroller and a custom-built MATLAB App Designer GUI. This project was the final deliverable for the ENGPHYS 3BB3 course at McMaster University.

## 🔧 System Overview

- **Objective**: Precisely regulate temperatures between 5°C and 45°C using a Peltier-based TEC and a thermistor.
- **Control Strategy**: Real-time PID control using serial communication between MATLAB and MSP430.
- **User Interface**: MATLAB GUI with real-time plotting, temperature setpoint entry, PID parameter tuning, and visual feedback.

## 🧰 Features

- Full **PID implementation** with dynamic gain switching based on error magnitude.
- **Sliding window averaging** (400 samples) for real-time smoothing.
- **UART communication** (115200 baud) between MATLAB and the MSP430.
- **PWM control** for heating and cooling via H-bridge driver (L298N).
- Live plotting of temperature data with a 10-second sliding time window.
- Intelligent “artificial booster” to enhance TEC cooling at low temperatures.
- Custom calibration for NTC thermistor using exponential fit with <±3°C error.

## 🛠 Hardware Used

- **Microcontroller**: MSP430G2553
- **TEC**: TEC1-12706 Peltier module
- **H-Bridge**: L298N dual motor driver
- **Thermistor**: muRata NCP21XV103J03RA
- **Power Supply**: 3.3V (logic), 12V (TEC)

## 💻 Software Stack

- **MATLAB App Designer**: for GUI, serial communication, and PID control
- **C (Code Composer Studio)**: for ADC sampling, UART handling, and PWM control
- **Real-time communication protocol**:
  - MATLAB sends trigger and control signals (`[sign; magnitude]`)
  - MSP sends back 400 ADC samples per cycle

## 🧪 Key Technical Insights

- MATLAB GUI includes logic for dynamic PID tuning and system visualization.
- UART-induced delay introduces ~11.5 kHz effective sampling rate.
- PWM duty cycle controlled via 8-bit resolution (0–255).
- Thermistor calibrated using:  
  `T(°C) = 67.5 * exp(-0.0001 * R) - 2`

## 📈 Results

- **Steady-state accuracy**: ±0.5°C
- **Temperature domain**: 5–45°C (with reliable TEC performance)
- **Performance boosts**: “Artificial booster” logic and PWM duty cycle fine-tuning for low temperatures
- **MATLAB GUI**: Stable, user-friendly interface with continuous data acquisition and dynamic visualization

## 🤝 Authors

- **Michael Zhikai Zhang** — [LinkedIn](https://www.linkedin.com/in/michael-zhang-077306154)
- **Cole Deroo**

## 📚 References

- [MSP430x2xx Family User Guide](https://www.ti.com/lit/ug/slau144k/slau144k.pdf)
- [NTC Thermistor – Ametherm](https://www.ametherm.com/thermistor/ntc-thermistors-steinhart-and-hart-equation)
- [Nyquist-Shannon Theorem](https://www.allaboutcircuits.com/technical-articles/nyquist-shannon-theorem-understanding-sampled-systems/)

---

> 🚀 This project demonstrates strong skills in embedded systems, real-time control, MATLAB integration, and collaborative engineering—ideal for QA Automation or Systems Integration roles.


