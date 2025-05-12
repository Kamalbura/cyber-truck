# ESP32 Cyber Truck RC Project

[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](https://choosealicense.com/licenses/mit/)
[![ESP32](https://img.shields.io/badge/ESP32-v4.4.0-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Arduino](https://img.shields.io/badge/Arduino-IDE-009D93.svg)](https://www.arduino.cc/)

![Project Banner](https://via.placeholder.com/1200x400?text=ESP32+Cyber+Truck+RC+Project)

## 📌 Overview

An advanced ESP32-based RC vehicle control system utilizing differential steering for smooth, car-like maneuverability. The project demonstrates expertise in embedded systems programming, PWM motor control, real-time signal processing, and wireless communication.

## 🎮 Key Features

- **Dynamic Differential Steering**: Implements sophisticated algorithm for smooth turning behavior similar to real vehicles
- **RC Signal Processing**: Utilizes interrupt-based timing for precise reading of RC receiver signals
- **Multiple Control Modes**: Supports various driving modes including proportional speed control and spin-in-place capabilities
- **Failsafe Protection**: Automatically stops motors if RC signal is lost to prevent runaway situations
- **LED Feedback System**: Integrated LED strip with multiple visualization modes for status indication
- **High-Current Motor Control**: Interfaces with BTS7960 H-Bridge drivers for controlling high-torque Johnson motors
- **Modular Architecture**: Clean code organization with separated functional components for easy maintenance and extension

## 🔧 Hardware Components

| Component | Specification | Purpose |
|-----------|---------------|---------|
| Microcontroller | ESP32 Development Board | Main control unit |
| Motor Drivers | BTS7960 H-Bridge (×2) | High-current motor control |
| Motors | Johnson 1000RPM (×4) | Vehicle propulsion |
| RC System | Standard 6-Channel Receiver | Remote control input |
| Power Supply | 7.4V-11.1V LiPo Battery | System power |
| LED System | WS2812B Addressable RGB LEDs | Visual feedback |
| Chassis | Custom Design | Vehicle structure |

## 🔌 Wiring Diagram

### RC Receiver to ESP32
- CH1 (Steering) → ESP32 pin 36
- CH3 (Throttle) → ESP32 pin 34
- Ground → ESP32 ground

### BTS7960 Motor Drivers to ESP32

#### Left Motors Driver
- RPWM (Forward) → ESP32 pin 25
- LPWM (Reverse) → ESP32 pin 26
- VCC → 5V or 3.3V from regulated source
- GND → ESP32 ground

#### Right Motors Driver
- RPWM (Forward) → ESP32 pin 32
- LPWM (Reverse) → ESP32 pin 33
- VCC → 5V or 3.3V from regulated source
- GND → ESP32 ground

## 📊 ESP32 System Architecture & Signal Flow

### Core Components Interaction
```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────────────────┐
│  RC Transmitter  │────▶│  RC Receiver    │────▶│  ESP32 Microcontroller      │
└─────────────────┘     └─────────────────┘     │  - Signal Processing         │
                                               │  - Differential Steering Logic│
                                               │  - PWM Generation             │
                                               │  - Failsafe Monitoring        │
                                               └───────────────┬───────────────┘
                                                               │
                      ┌──────────────────────────────────────┬─┴─┬──────────────────────────────────┐
                      │                                      │   │                                  │
              ┌───────▼──────┐                    ┌──────────▼───▼──────────┐            ┌──────────▼─────────┐
              │ BTS7960      │                    │ BTS7960                 │            │ LED Control System  │
              │ Left Motors  │                    │ Right Motors            │            │ (FastLED/NeoPixel)  │
              └───────┬──────┘                    └──────────┬──────────────┘            └──────────┬──────────┘
                      │                                      │                                      │
              ┌───────▼──────┐                    ┌──────────▼──────────────┐            ┌──────────▼──────────┐
              │ Left Motors  │                    │ Right Motors            │            │ WS2812B RGB LEDs    │
              │ (Front/Back) │                    │ (Front/Back)            │            │ (Status/Effects)    │
              └──────────────┘                    └───────────────────────┬─┘            └─────────────────────┘
                                                                         │
┌─────────────────────────────────────────────────────────────────────────┴───────────────────────────────────┐
│                                       Vehicle Movement & Visual Feedback                                     │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

### ESP32 Signal Processing Flow
```
                              ┌───────────────────────────────────────┐
                              │              ESP32                    │
                              │                                       │
┌─────────────────┐           │    ┌───────────────────────────────┐  │
│  RC Receiver     │──CH1─────┼───▶│ Interrupt Handlers            │  │
│  Channels        │──CH3─────┼───▶│ (Precise RC Signal Timing)    │  │
└─────────────────┘           │    └───────────────┬───────────────┘  │
                              │                    │                  │
                              │    ┌───────────────▼───────────────┐  │
                              │    │ Signal Validation & Mapping   │  │
                              │    │ (Converts RC values to usable │  │
                              │    │  throttle/steering values)    │  │
                              │    └───────────────┬───────────────┘  │
                              │                    │                  │
                              │    ┌───────────────▼───────────────┐  │
                              │    │ Differential Steering Logic   │  │
                              │    │ (Calculates individual motor  │  │
                              │    │  speeds for smooth turning)   │  │
                              │    └─┬─────────────────────────┬───┘  │
                              │      │                         │      │
┌─────────────────┐           │    ┌─▼─────────────┐  ┌────────▼───┐  │    ┌─────────────────┐
│  LED Control    │◀──────────┼────│ LED Pattern   │  │ PWM Signal │  │───▶│ BTS7960 Drivers │
│  (Status/Mode)  │           │    │ Generation    │  │ Generation │  │    │ (Motor Control) │
└─────────────────┘           │    └───────────────┘  └────────────┘  │    └─────────────────┘
                              │                                       │
                              └───────────────────────────────────────┘
```

## 💡 LED Lighting System

The Cyber Truck includes a sophisticated LED feedback system that provides visual status indication and aesthetic effects.

### LED Hardware Configuration
- **LED Type**: WS2812B Addressable RGB LEDs (NeoPixels)
- **Control Pin**: GPIO 13
- **Number of LEDs**: 12
- **Power**: 5V, driven directly from ESP32

### LED Functionality Matrix

| Mode | Description | Operational Pattern | Associated RC Channel |
|------|-------------|---------------------|----------------------|
| 0 | All Off | All LEDs disabled | AUX2 position 1 |
| 1 | Speed Indicator | Color gradient based on motor speed | AUX2 position 2 |
| 2 | Rainbow Pattern | Continuous color cycling animation | AUX2 position 3 |
| 3 | Direction Indicators | Blue (forward), Red (reverse), Yellow (turning) | AUX2 position 4 |
| 4 | Chase Effect | Single LED chase around perimeter | AUX2 position 5 |
| Failsafe | Warning Indicator | Rapid red flashing when signal lost | Automatic |

### LED Integration Logic

```cpp
// LED control based on vehicle state
void updateLEDs() {
  switch (ledMode) {
    case 0: // All off
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      break;
      
    case 1: // Speed indicator - color based on throttle
      {
        int speed = max(abs(leftSpeed), abs(rightMotorSpeed));
        CRGB color = CRGB(speed, 255-speed, 0); // Red to Green based on speed
        fill_solid(leds, NUM_LEDS, color);
      }
      break;
    
    // Additional modes...
  }
}
```

## 🧠 ESP32 Control Architecture

### Core ESP32 Functionality
- **RC Signal Reading**: Hardware interrupt-based pulse timing (1-2ms PPM signals)
- **Motor Control**: 4-channel PWM generation for BTS7960 H-Bridge drivers
- **Differential Steering**: Dynamic speed calculation based on throttle and steering inputs
- **LED Animation**: Fast parallel processing of lighting effects
- **Failsafe Monitoring**: Continuous validation of input signals with timeout detection

### ESP32 Pin Assignment

| Function | GPIO Pin | Description |
|----------|----------|-------------|
| RC Channel 1 | 36 | Steering input (analog pin) |
| RC Channel 3 | 34 | Throttle input (analog pin) |
| Left Motors Forward | 25 | PWM control to BTS7960 |
| Left Motors Reverse | 26 | PWM control to BTS7960 |
| Right Motors Forward | 32 | PWM control to BTS7960 |
| Right Motors Reverse | 33 | PWM control to BTS7960 |
| LED Data | 13 | Addressable LED control |
| Additional RC Channels | 32, 33 | Mode selection and features |

### ESP32 Processing Workflow

1. **Signal Acquisition**
   - Capture RC signal pulses via interrupts
   - Convert pulse timing to normalized values (-255 to 255)

2. **Control Logic Processing**
   - Apply deadzone filtering to remove jitter
   - Calculate differential steering values
   - Determine appropriate LED patterns

3. **Output Generation**
   - Generate PWM signals for motor control
   - Update LED strip with current pattern
   - Send debug data via Serial (if active)

4. **Safety Monitoring**
   - Check for signal timeout conditions
   - Implement smooth transitions between states
   - Prevent motor driver conflicts

## 🚀 Getting Started

### Prerequisites
- Arduino IDE (1.8.13 or later)
- ESP32 board package installed in Arduino IDE
- Required libraries:
  - FastLED
  - Arduino.h

### Installation

1. Clone this repository:
```bash
git clone https://github.com/yourusername/esp32-cyber-truck.git
```

2. Open the Arduino IDE and load the `esp32_rc_car.ino` file

3. Select the appropriate ESP32 board from Tools > Board menu

4. Connect your ESP32 board via USB

5. Compile and upload the sketch

## 📝 Usage Instructions

1. Connect hardware according to the wiring diagram
2. Power on the RC transmitter
3. Power on the vehicle
4. Use the transmitter controls:
   - Left stick: Forward/Reverse throttle
   - Right stick: Left/Right steering
   - Channel 5/6: Additional features

## 🔬 Technical Achievements

- **Real-time Signal Processing**: Implemented high-precision interrupt-based RC signal reading
- **Advanced Control Algorithms**: Developed proportional differential steering with smooth transitions
- **Efficient Memory Usage**: Optimized code for ESP32's limited memory environment
- **Failsafe Implementation**: Created robust error detection and handling mechanisms

## 🔮 Future Enhancements

- [ ] Add MPU6050 for gyroscopic stabilization
- [ ] Implement Bluetooth control option via smartphone app
- [ ] Add telemetry data transmission
- [ ] Develop autonomous navigation capabilities
- [ ] Create web interface for configuration settings

## 📜 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👨‍💻 Author

Your Name - [Your GitHub Profile](https://github.com/yourusername)

## 🙏 Acknowledgments

- ESP32 Community for their excellent documentation
- Arduino community for library support
- [Add any other acknowledgments here]

---

*Created as part of an embedded systems engineering project, this repository demonstrates practical application of microcontroller programming, PWM motor control, and real-time systems design.*

![Project Demo](https://via.placeholder.com/800x400?text=Project+Demo+Video+Coming+Soon)
