<h1 align="center">Design Folder</h1>

<p align="center">
  <img src="../assets/icons/schematic.png" alt="Schematic Overview" width="200"/>
</p>

## Table of Contents
- [Table of Contents](#table-of-contents)
- [📖 Overview](#-overview)
- [🛠️ Implementation Note](#️-implementation-note)
- [📂 Schematic Files](#-schematic-files)
  - [🔌 1. Power Electronics Schematic](#-1-power-electronics-schematic)
  - [Final PCB for Power Drive](#final-pcb-for-power-drive)
  - [🖐️ 2. Embedded System Schematic](#️-2-embedded-system-schematic)
  - [Final PCB for Glove Control](#final-pcb-for-glove-control)
- [🛠️ Wiring Instructions](#️-wiring-instructions)
- [🔗 Useful Links](#-useful-links)

## 📖 Overview  

This folder contains two important schematic files related to the design of the Robotic Arm project. Each schematic serves a specific purpose and is crucial for the development and integration of the power electronics and embedded systems.
> 📄 All schematics are provided in PDF format for easy viewing and printing.
> All the voltages mentioned are in DC unless specified.
> Designed and documented by **IoRT Club** - Embedded & Power Crew.


## 🛠️ Implementation Note

You can implement the electronics in two ways:

- **Custom PCB**:  
  Build a dedicated PCB by referring to the provided schematics for a more compact and reliable design.

  You can design the PCB using free tools such as:
  - [KiCad](https://www.kicad.org/) — Open-source, highly customizable PCB design software.
  - [EasyEDA](https://easyeda.com/) — Browser-based and beginner-friendly PCB design platform.

  > **Note**: Both tools are free to use (with some limitations).  
  After completing your design, export the files in **Gerber format** and submit them to a PCB fabrication vendor for manufacturing.

- **Perfboard / Breadboard**:  
  Alternatively, for prototyping or quick builds, you can assemble the circuits manually on a perfboard by carefully following the schematic connections. 
  Using a perfboard is recommended if manufacturing a custom PCB is not feasible or practical.


> **Note**: Ensure proper wire management and soldering practices if you choose the perfboard method to maintain signal integrity and minimize errors.


## 📂 Schematic Files

### 🔌 1. Power Electronics Schematic 

- **File Name**: [Schematic_Power-Drive-Gesture-Controlled-Robotic-Arm_2025-04-26](Schematic_Power-Drive-Gesture-Controlled-Robotic-Arm_2025-04-26.pdf)
- **Description**: This schematic implements the power drive for the robotic arm, including the configuration for servos and motors. It outlines how power is managed and distributed to ensure optimal performance of the robotic arm's movements.
  - Handles the actuation part of the robotic arm including control for servos and a NEMA17 stepper motor.
  - Features motor driving using DRV8825 for stepper control and PCA9685 for multi-servo PWM output.
  - Power management done using LM2596 buck converter and dual LM7805 regulators.
- **Key ICs**:
  - ESP32-WROOM32
  - DRV8825 Stepper Motor Driver
  - PCA9685 16-Channel PWM Driver
  - LM2596 Adjustable Buck Converter
  - LM7805 Voltage Regulators
- **Notes**:
  - Main voltage input via XT60 connector (e.g., 12V supply).
  - Separate regulation for logic and motor sections to avoid noise.

> ⚠️ **Power Safety Warning**  
> 
> The Power Drive system is designed to operate using a **12V DC SMPS (Switched-Mode Power Supply)** rated at a minimum of **2.5A**.  
> 
> Please exercise extreme caution while handling the power system:
> - Ensure the SMPS is properly rated and from a reputable manufacturer.
> - Always verify polarity before connecting the supply to the circuit.
> - Never work on the circuit while the SMPS is powered ON.
> - Use appropriate fuses or protection circuits if possible.
> 
> 🔥 Mishandling the power supply can cause **serious damage to components** or **pose a fire hazard**.  
> Double-check all connections before powering the system.

### Final PCB for Power Drive
<div align="center">
<!-- PCB for Power Drive -->
<img src="../assets/images/Screenshot 2025-02-11 091135.png" 
     alt="Final PCB for Power Drive" 
     width="800" 
     style="border: 2px solid #ccc; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.2); transition: transform 0.3s; margin-bottom: 30px;"
     onmouseover="this.style.transform='scale(1.05)';"
     onmouseout="this.style.transform='scale(1)';"/>
</div>

### 🖐️ 2. Embedded System Schematic 

- **File Name**: [Schematic_Glove-Gesture-Controlled-Robotic-Arm_2025-04-26](Schematic_Glove-Gesture-Controlled-Robotic-Arm_2025-04-26.pdf)
- **Description**: This schematic details the data acquisition part of the robotic arm, which captures user gestures through a glove interface. It is designed to facilitate communication between the user's gestures and the robotic arm's movements.
  - Captures the user gestures via an ESP32-WROOM32 module, two MPU6050 IMUs, and flex sensors.
  - Features voltage regulation using AMS1117 (3.3V) and LM7805 (5V) to power sensors reliably.
  - Includes TTP223 touch sensors for auxiliary control inputs.
- **Key ICs**:
  - ESP32-WROOM32
  - AMS1117-3.3 Voltage Regulator
  - LM7805 Voltage Regulator
  - Two MPU6050 Modules
  - Flex Sensors
- **Notes**:
  - Operates at 3.3V logic levels.
  - Input supply is 9V regulated down. Ensure the input provided is above 5V

### Final PCB for Glove Control
<div align ="center">
<!-- Final GLove Control PCB -->
<img src="../assets/images/Screenshot 2025-02-11 091319.png" 
     alt="Final Glove Control PCB" 
     width="800" 
     style="border: 2px solid #ccc; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.2); transition: transform 0.3s; margin-top: 30px;"
     onmouseover="this.style.transform='scale(1.05)';"
     onmouseout="this.style.transform='scale(1)';"/>
</div>

## 🛠️ Wiring Instructions
- Ensure the all components are upto the rating as mentioned in the [BOM](../docs/Bill_of_Materials_Electrical.pdf)
- Make sure the pinout of your component matches the one provided in the schematic
- If a perfboard based design if being followed ensure that each voltage levels have different rails (think of it as a path for the current), the ground can occupy a single rail.
-  [🔌 1. Power Electronics Schematic ](#-1-power-electronics-schematic) — 
   -  ESP32-WROOM32:
         - GPIO Pins to control servos via PCA9685.
         - SDA (GPIO21) → SDA of PCA9685
         - SCL (GPIO22) → SCL of PCA9685 

   - PCA 9685
      - Set the buck converter(LM2596) to output a steady voltage of 5.5V
      - The servos are connected to channel 0,1 and 2

   - DRV8825 (Stepper Motor Driver):  
       - DIR (Direction) and STEP pins connected to GPIOs of ESP32 for motor control.
       - Enable pin if required can be connected to a GPIO pin

- [🖐️ 2. Embedded System Schematic ](#️-2-embedded-system-schematic) —
  - MPU6050 IMUs:
    - SDA (GPIO21) and SCL (GPIO22) for I2C communication.
    - One of the MPU6050 IMU has its ADD pin (pin no 7) pulled high to _+3.3V_ this so that both the MPU sensors have unique I2C addresses.

  - Flex Sensors and TTP223 Touch Sensors:
    - Flex sensors and touch sensors connected to the ESP32 analog or digital inputs based on your design.

## 🔗 Useful Links
- [PCA9685 Datasheet and Pinout](https://www.espboards.dev/sensors/pca9685/#specs)
- [DRV8825 Datasheet and Pinout](https://www.makerguides.com/drv8825-stepper-motor-driver-arduino-tutorial/)
- [ESP32 Datasheet and Pinout](https://randomnerdtutorials.com/getting-started-with-esp32/)
- [MPU6050 Datasheet and Pinout](https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/)
- [KiCAD Basics](https://www.youtube.com/playlist?list=PLn6004q9oeqGl91KifK6xHGuqvXGb374G)
- [Soldering Tutorial — 1](https://www.instructables.com/A-Comprehensive-Guide-to-Soldering-Techniques-Tool/)
- [Soldering Tutorial — 2](https://www.sciencebuddies.org/science-fair-projects/references/how-to-solder)
