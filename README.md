![IMG-20250625-WA0026](https://github.com/user-attachments/assets/78f6c05a-cd18-4aa9-af28-e22b9231c0d9)
🚗 ParkMate: Intelligent Autonomous Parking System
ParkMate is a graduation project that demonstrates a fully autonomous self-parking car using computer vision, embedded systems, and wireless control. The system detects empty parking spots and navigates the vehicle safely into them without human intervention.

🔧 Features
Real-time parking slot detection using YOLOv8 and Pi Camera on Raspberry Pi 4

Motor and sensor control using ESP8266, communicating via UART

Integration of 6 ultrasonic sensors and 2 IR sensors for obstacle avoidance and alignment

Dual-mode control via a Wi-Fi mobile app (manual & automatic)

Embedded drivers: GPIO, timer, ultrasonic sensor, UART communication

Smooth execution of parallel and perpendicular parking maneuvers

🧠 Technologies Used
Python, OpenCV, YOLOv8 (Ultralytics)

C/C++ for ESP8266 (Arduino Framework)

Mobile app over IP using Blynk (can be customized)

Real-time embedded control logic

📦 Hardware
Raspberry Pi 4 Model B

ESP8266 (NodeMCU)

HC-SR04 Ultrasonic Sensors

IR Line Sensors

L298N Motor Driver

Pi Camera v1.3
