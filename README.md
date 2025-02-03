## 📜 README - GyroBot

### 🛠️ Introduction
**GyroBot** is a project based on the **MPU6050** and servomotors designed to stabilize a structure by maintaining a target angle. It uses a **PID controller** to dynamically adjust the angle and ensure balance.

### 🎯 Objective
The main goal is to **maintain a stable position** by adjusting the servomotors' angle according to the IMU (**Inertial Measurement Unit**) readings. The project implements a PID control system to minimize errors and stabilize the system's angle.

---

## 🏗️ Required Hardware
- **Arduino (Uno, Mega, etc.)**
- **MPU6050** (Gyroscope & Accelerometer)
- **2 servomotors** (e.g., SG90 or MG995)
- **Wires and power supply**

---

## 🔍 How It Works

### 1️⃣ **Initialization**
- The **Arduino** initializes communication with the **MPU6050** via **I2C**.
- The **servomotors** are attached to the predefined pins.

### 2️⃣ **Reading Data from MPU6050**
- The angle is computed using **atan2** on acceleration values.
- A **moving average filter** is applied to smooth out the measurements.

### 3️⃣ **PID Control**
- A **PID (Proportional - Integral - Derivative) controller** adjusts the servos to reduce the error between the current angle and the target angle.
- The PID output is limited to prevent extreme values.

### 4️⃣ **Servo Control**
- The PID output angle is used to **adjust the servomotor positions**.
- A **smooth interpolation** ensures gradual transitions between positions.

### 5️⃣ **Motion Transition Management**
- The system alternates between **stabilization** and **forward motion** using a counter.
- A **sigmoid function** is used to smooth the angle transitions.

---

## 🚀 How to Use
1. **Connect the hardware** to the Arduino.
2. **Upload the code** using the Arduino IDE.
3. **Open the Serial Monitor** (115200 baud) to observe the angle and PID corrections.
4. **Watch the servos** adjust the balance according to the movements.

---

## ⚙️ Possible Improvements
- Implement **complementary filtering** for better accuracy.
- Optimize **PID coefficients** using an **auto-tune algorithm**.
- Integrate **Bluetooth control** to dynamically adjust the target angle.

---

## 📌 Conclusion
**GyroBot** is a system based on an **MPU6050 and servos**, using a **PID controller** to stabilize an angle. This project can be improved and adapted for applications like **self-balancing robots** or **gyrostabilized systems**.

🚀 *Time to build and experiment!* 🎮
