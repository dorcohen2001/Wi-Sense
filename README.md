Wi-Sense — Wi-Fi CSI Motion-Detection Suite
Welcome to Wi-Sense, an advanced motion detection ecosystem that leverages Wi-Fi Channel State Information (CSI) extracted from ESP32 microcontrollers. By analyzing variations and disruptions in subcarrier amplitudes caused by physical movement, this suite transforms standard Wi-Fi signals into a highly sensitive, localized spatial radar.

This repository contains two distinct architectural implementations of the system, tailored for different deployment needs.

📂 Repository Architecture
The project is split into two primary modules, each featuring its own self-contained codebase and dedicated README.md detailing specific setup and operation instructions

1. 🖥️ wi-sense-desktop
Concept High-performance data streaming and deep visualization.

How it works The ESP32 receiver acts as a gateway, capturing raw CSI packets and streaming them over a serial connection directly to a host PC. A rich Python pipeline handles advanced filtering (Savitzky-Golay smoothing, adaptive threshold calculations) and renders a detailed, real-time graphical dashboard.

Best for Development, real-time signal diagnostics, historical data logging (.jsonl), and academic research into CSI behavior.

Real-Time Visualization Dashboard
<img width="743" height="543" alt="Screenshot 2026-05-29 135259" src="https://github.com/user-attachments/assets/49c06963-4bf8-493f-b39f-878bd6749fd2" />

2. 📱 wi-sense-edge
Concept Fully embedded, standalone edge computing.

How it works No PC or Python script required. All digital signal processing, variance filtering, and threshold analysis are performed directly on the ESP32 chip (Edge Processing). The board generates its own local Wi-Fi network (Wi-Sense-Net) and hosts a lightweight web server.

Best for Low-power standalone applications, quick field deployment, and monitoring motion alerts directly from any mobile web browser via portable power banks.

Real-Time Visualization Dashboard
<img width="476" height="660" alt="image" src="https://github.com/user-attachments/assets/4dad4212-d181-4958-b223-77a6439cca60" />





🚀 Getting Started
To fully understand, deploy, and configure either version, navigate into your directory of choice and follow the detailed step-by-step documentation provided within

Go to Desktop Implementation 👉 wi-sense-desktopREADME.md

Go to Standalone Edge Implementation 👉 wi-sense-edgeREADME.md

🛠️ General Requirements & Hardware
Both systems utilize the same base hardware architecture

2x ESP32 Development Boards (configured as a TransmitterReceiver pair).

Arduino IDE with ESP32 Core support.

Portable power sources (e.g., power banks or USB wall adapters) for completely untethered hardware nodes.
