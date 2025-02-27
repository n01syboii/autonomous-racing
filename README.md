# Autonomous Car using YDLidar and PID Steering Control

This project implements a basic autonomous car system using a YDLidar sensor for obstacle detection and navigation, paired with a Rosmaster robot car for movement and steering control. The system leverages vector addition, angle normalization, and a PID controller to dynamically adjust the steering angle based on real-time LiDAR data.

---

## Features

✅ Real-time obstacle detection using YDLidar  
✅ Dynamic steering adjustment using PID control loop  
✅ Full 360° LiDAR scan with configurable range (up to 200 meters)  
✅ Continuous forward motion with automatic steering corrections

---

## Hardware Used

- 🚗 Rosmaster Robot Car (or compatible differential drive robot platform)
- 🔧 YDLidar (Time-of-Flight LiDAR sensor)
- 💻 Raspberry Pi / Jetson Nano / compatible Linux system to run the Python code

---

## How It Works

### 1. LiDAR Scanning

- The YDLidar sensor scans the environment in 360°.
- Distance and angle data are collected in real-time.
- Configured for a maximum range of 200 meters.

### 2. Vector Addition & Normalization

- LiDAR points in the front-facing range (105° to 255°) are used.
- These points are converted to x, y coordinates, and a resultant vector is computed.
- The resultant angle is normalized to fit within 0° to 180°, which maps to the steering servo range.

### 3. PID Steering Control

- A PID controller calculates the optimal steering correction to align the car with the desired path.
- The steering servo angle is dynamically adjusted based on:
  - Proportional error (current deviation from target)
  - Integral error (sum of past errors)
  - Derivative error (rate of change of error)

### 4. Motor Control

- Motors are set to move the car forward at constant speed.
- Steering is continuously corrected based on real-time LiDAR feedback.

---

## Installation

### Requirements

- Python 3.x
- YDLidar Python SDK
- Rosmaster Library
- Numpy

### Install Dependencies

```bash
pip install numpy matplotlib
```

### YDLidar SDK

Make sure the YDLidar SDK is properly installed. Refer to:
👉 https://github.com/YDLIDAR/YDLidar-SDK

---

## Usage

1. Connect your YDLidar sensor to the system.
2. Ensure the port (usually /dev/ydlidar) is correctly recognized.
3. Run the code:

   ```bash
   python autonomous_car.py
   ```

<<<<<<< HEAD
=======

>>>>>>> fce963f7e7518e0d6433835c8a7173ab4b20aa04
4. The car will begin moving forward, continuously scanning and adjusting its steering to avoid obstacles.

---

## Configuration

| Parameter     | Default | Description                               |
| ------------- | ------- | ----------------------------------------- |
| Kp            | 0.61    | Proportional gain for steering correction |
| Ki            | 0.0     | Integral gain (set to 0 for now)          |
| Kd            | 1.18    | Derivative gain for damping oscillations  |
| MaxRange      | 200.0 m | Maximum LiDAR detection range             |
| ScanFrequency | 10 Hz   | Frequency of LiDAR scans                  |

---

## Important Notes

- This system is designed for indoor/outdoor environments with obstacles.
- The car assumes a clear front-facing path and adjusts steering based on detected obstacles.
- Emergency stop and reverse logic can be added if necessary.

---

## File Structure

```
.
├── autonomous_car.py # Main code file
└── README.md # Documentation (this file)
```

---

## Future Improvements

- Add emergency stop for close-range obstacles.
- Implement adaptive speed control based on obstacle proximity.
- Add logging for better debugging.
- Extend support for side obstacles (lateral scanning).

---
