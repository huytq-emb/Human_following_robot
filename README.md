
# Human-Following Mobile Robot 🚶‍♂️🤖

A graduation thesis project that focuses on the **design, construction, and programming** of a mobile robot capable of **following a human target** using **deep learning-based computer vision**, obstacle avoidance, and omnidirectional mecanum wheels.

## 📌 Project Overview

This robot is designed to autonomously follow a person using a mounted camera and object detection algorithms. It can move omnidirectionally thanks to mecanum wheels, and it uses real-time image processing combined with distance sensors for collision avoidance.

## 🧠 Features

- Human detection and tracking using **MobileNet-SSD**.
- Obstacle avoidance using **ultrasonic**, **infrared**, and **Lidar** sensors.
- PID-controlled motion with **4 DC encoder motors**.
- Flexible movement in all directions (forward, backward, sideways, diagonal, rotation).
- Real-time processing via **Raspberry Pi 3B+** and **STM32F103RCT6**.

## 🛠️ Hardware Used

- **Chassis**: Custom-designed frame with SolidWorks
- **Motors**: 4x DC motors with encoders (JGB37-520)
- **Wheels**: 4x Mecanum wheels
- **Microcontrollers**:
  - STM32F103RCT6 (for low-level motor control)
  - Raspberry Pi 3B+ (for image processing)
- **Sensors**:
  - TF-Luna Lidar
  - HC-SR04 ultrasonic sensor
  - E18-D80NK infrared sensor
- **Camera**: Aukey PC-W3 1080P webcam
- **Motor driver**: L298N
- **Power Supply**: 4S 2200mAh Li-Po Battery + Xiaomi 10,000mAh Powerbank

## 🔧 Software Stack

- **Languages**: C (STM32), Python (Raspberry Pi)
- **Libraries/Frameworks**:
  - OpenCV
  - TensorFlow / MobileNet-SSD
  - PySerial / UART / MQTT for communication
- **PID Tuning**: Ziegler-Nichols, IMC, Trial-and-error

## 🧪 How It Works

1. **Human Detection**:
   - The Raspberry Pi captures video using a webcam.
   - Frames are processed with MobileNet-SSD to detect and identify the person.

2. **Object Tracking**:
   - If a person is detected, the robot calculates their position in the frame.
   - Based on deviation from the center, the robot adjusts its direction and speed.

3. **Obstacle Avoidance**:
   - Distance sensors monitor the environment.
   - If an obstacle is detected, the robot either stops or changes direction.

4. **Motion Control**:
   - Commands are sent to STM32 to control the mecanum wheels using PID.
   - Movement types include forward, backward, lateral, diagonal, and rotation.

## 🧪 Demo & Environment Setup

### Raspberry Pi Setup
```bash
sudo apt update && sudo apt upgrade
sudo apt install python3-pip python3-opencv
pip3 install tensorflow numpy pyserial imutils
```

### Running the Demo
1. Clone the repository and navigate to the vision folder.
2. Connect the hardware.
3. Run file:

human_following.py on computer.
bridge.py on raspberry.


### STM32 Setup
- Flash the firmware in `firmware/STM32/` using STM32CubeProgrammer.
- Ensure UART communication matches between Pi and STM32.

## 🔌 Hardware Wiring Overview

- **DC Motors** ← L298N ← STM32
- **L298N Motor Driver** ← 12V Battery
- **TF-Luna** ← UART to Raspberry Pi
- **Ultrasonic/IR Sensors** ← GPIO Raspberry Pi
- **Webcam** ← USB Raspberry Pi

## 🚀 Future Improvements

- Upgrade detection using YOLOv5 for better precision.
- Add voice command capabilities.
- Integrate SLAM for full autonomous navigation in unknown environments.

## 📜 License

This project is developed for educational purposes at HCMUTE (Ho Chi Minh City University of Technology and Education). All rights reserved.

