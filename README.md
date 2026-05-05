Avant-Firmware
Firmware repository for Avant Actuation's motorized prosthetic leg systems. This codebase handles sensor integration, motor control, and gait pattern prediction to deliver accessible and affordable prosthetic actuation.

Hardware Overview
Microcontroller: Arduino Nano ESP32

Sensors: MPU6050 Gyroscopes (Synchronized Array)

Actuators: High-torque servos

Key Features
Gait Prediction: Integrated neural network processing to predict and adapt to user gait patterns.

Sensor Fusion: Real-time data processing from the gyroscope array for precise spatial awareness.

Responsive Actuation: Low-latency motor control for smooth, natural movement.

Getting Started
Clone the repository:

Bash
git clone https://github.com/pwtoothless/Avant-Firmware.git
Open the project in the Arduino IDE.

Ensure the Arduino ESP32 board definitions are installed in your Board Manager.

Install the necessary dependencies for the MPU6050 and servo control.

Connect your board, select the appropriate COM port, compile, and upload.

Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

License
None
