# UGV Controller on the ESP32 platform

## Decription

This repository contains the source code for the low-level control on the ECU-UGV. It is implemented using the Arduino IDE compiler along with libraries to support PID control and TFmini Plus range measuring.

A FlySky remote controller is used to remotely teleoperate the UGV. The left and right analog sticks on the controller operate the left and right tracks of the UGV, respectively. The UGV will scan in front and behind it using the TFmini Plus modules and willstop if it detects an object in close proximity to it (less than 1 metre).

## Hardware

- ESP32 Development board
- TFmini Plus (I2C mode)
- FlySky remote controller
- UGV platform

### ESP32 Pin Diagram

![alt text](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/_images/esp32-devkitC-v4-pinout.png)

## Logic Overview

### Inputs

#### FlySky remote (PWM input)

The ESP32 will read the PWM inputs from the FlySky remote analog sticks and scale the duty cycle measured to an integer between -255 and 255.

#### Rotary Encoders (Digital input)

The encoders will measure the speed of the motors. They pulse 1024 times per revolution, however the encoders are attached to the motor axles by a belt, gearing the system up by 2.5. Thus, there are actually 2560 pulses per *motor* revolution. This input will be used in the PID calculations.

#### TFmini Plus (I2C)

- IR Range sensors
- 0.1m to 12m range
- UART or I2C communication. I2C is used in this application due to less wiring when using multiple modules.
- Used for close range proximity measurements for obstacle avoidance system

### Outputs

#### Motor controller (PWM or Analog ???)

- WIP

## Libraries Used

- [Offical ArduinoLibrary](https://www.arduino.cc/reference/en/libraries/)
- [TFMini-Plus-I2C-API](https://github.com/budryerson/TFMini-Plus-I2C) by Bud Ryerson
- [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library) by br3ttb

### Authors

- Brendan O'Malley
