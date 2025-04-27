<h1 align="center">Gesture-Controlled Robotic Arm</h1>

<p align="center">
  <img src="assets/icons/3-dof-arm.png" alt="3-DOF Arm" width="200">
</p>

<details>
  <summary><h2>📚 Table of Contents (Click to Expand)</h2></summary>

- [📖 Project Description](#-project-description)
- [✨ Features](#-features)
- [⚙️ Installation Instructions](#️-installation-instructions)
- [🔧 Use Cases](#-use-cases)
- [🤝 Contributing](#-contributing)
- [📜 License](#-license)
- [🙏 Acknowledgments](#-acknowledgments)
  - [🤖 Meet the Team! 🤝](#-meet-the-team-)
- [📧 Contact Information](#-contact-information)

</details>

---

## 📖 Project Description  

The **Gesture-Controlled Robotic Arm** is designed to automate repetitive tasks using hand gestures. The robotic arm consists of **four links**: a **base, two links, and an end effector**, enabling **pick-and-place functionality**.  

Control is achieved via a **glove-based gesture system**, equipped with **flex sensors and an accelerometer**, translating hand movements into precise robotic commands. The arm is powered directly from a **single-phase AC source**, eliminating the need for batteries and enhancing efficiency.  

---

## ✨ Features  

✅ **Precise movement**  
✅ **Gesture-based control** using flex sensors and an accelerometer  
✅ **Pick-and-place functionality** for automation tasks  
✅ **Intuitive user interface** for seamless control  

---

## ⚙️ Installation Instructions  

- 🛠️ For installation instructions and to download the Arduino IDE, visit the [Arduino IDE Download Page](https://www.arduino.cc/en/software).
- Add the ESP32 Library to the ide [Instructions](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
- To set up the project, follow these steps: 
- Clone the repository:
- ```bash
  git clone https://github.com/Robo-Linkers/Gesture-controlled-robotic-arm.git
  ```
- Install any necessary dependencies [REQUIREMENTS](requirements.txt)
- Navigate to the project directory:
- ```cd Gesture-controlled-robotic-arm```
- 
---

## 🚀 Usage Instructions  
- After setting up the hardware and software, wear the glove equipped with sensors, power on the robotic arm, and start performing gestures to control its movements.
  ### Components
  - Robotic Arm: 4-link structure with servos.
  - Glove Interface: Equipped with flex sensors and an accelerometer
  - Microcontroller: ESP32 or compatible board for processing input signals
  - Power Supply: AC power source for operation.
  ### Designs
  - Refer to the designs folder for schematic diagrams and design documents related to the glove and robotic arm.
  ### Testing
  - The tests folder contains Arduino sketches for testing individual components, including:
    - Flex sensors
    - Dual MPU6050
    - Power electronics (motors, servos)

---

## 📂 Folder Structure  
```bash
Gesture-controlled-robotic-arm/
├── .gitignore
├── LICENSE
├── README.md
├── assets/
│   └── icons/
│       ├── 3-dof-arm.png
│       ├── binary-code.png
│       ├── docs.png
│       ├── pcb-board.png
│       └── schematic.png
├── designs/
│   ├── DESIGN.MD
│   ├── Schematic_Glove-Robotic-Arm_2025-01-13.pdf
│   └── Schematic_Power-Drive-Controlled-Robotic-Arm_2025-01-13.pdf
├── logs/
│   ├── glove_control.log
│   └── power_drive.log
├── src/
│   ├── Glove.ino
│   └── Power_Drive.ino
└── tests/
    ├── Embedded Systems/
    │   ├── COMPLETE/
    │   │   └── COMPLETE.ino
    │   ├── DUAL-MPU6050/
    │   │   └── DUAL-MPU6050.ino
    │   └── FLEX-SENSORS/
    │       └── FLEX-SENSORS.ino
    └── Power Electronics/
        ├── complete-dynamic/
        │   └── complete-dynamic.ino
        ├── complete/
        │   └── complete.ino
        ├── nema_17_test/
        │   └── nema_17_test.ino
        └── servo_test/
            └── servo_test.ino
    └── TESTS.MD
  
```
---

## 🔧 Use Cases  

💡 **Manufacturing** → Automating repetitive tasks like assembly & packaging  
🏥 **Healthcare** → Assisting in surgeries or rehabilitation exercises  
🎓 **Education** → Demonstrating robotics concepts in academic settings  
🧪 **Research** → Exploring human-robot interaction and automation  

---

## 🤝 Contributing  
- Contributions are welcome! Please feel free to submit a pull request or open an issue if you have suggestions or improvements, for more details refer [CONTRIBUTING](CONTRIBUTING.md).

---

## 📜 License  

This project is licensed under the **MIT License**. Refer to the [LICENSE](LICENSE) file for more details.  

---

## 🙏 Acknowledgments  

### 🤖 Meet the Team! 🤝

- [Namitha Madhu](https://www.linkedin.com/in/namitha-madhu-4934a8276/) - Embedded Lead 🚀
    - [Madhubala M](https://www.linkedin.com/in/madhubala-m-0b66752bb/) - Embedded Co-Lead 💡
    - [Bhagyashree M](https://www.linkedin.com/in/bhagyashree-m-a21853343/) - Hardware Integration Engineer 
    - [Keerthivasan](https://www.linkedin.com/in/keerthivasansv/) - The Code Wizard 🧙‍♂️ 

- [Gomathi Manisha](https://www.linkedin.com/in/gomathi-manisha-a-3894b8285) - Project Co-Lead 🚀

- [Anmol Krishh](https://www.linkedin.com/in/anmolkrish/) - Project Lead and Power Electronics Lead ✨⚡️
    - [Shivani K.C](https://www.linkedin.com/in/shivani-k-c-543513276/) - Power Electronics Co-Lead ⚡️

- [Mithill Prabhu](https://www.linkedin.com/in/mithill-prabhu/) - Design Lead ⚙️

---

## 📧 Contact Information  
- For any inquiries or feedback, please contact us at [GitHub](https://github.com/Robo-Linkers/Gesture-controlled-robotic-arm).
