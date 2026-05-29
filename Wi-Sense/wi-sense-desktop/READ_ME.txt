Wi-Sense — Adaptive CSI Motion-Detection System
Wi-Sense is a real-time motion detection system that utilizes Wi-Fi Channel State Information (CSI) extracted from ESP32 microcontrollers. By analyzing variations in subcarrier amplitudes caused by environmental disruptions, the system can accurately detect human movement and visualize spatial variance through a dynamic Python dashboard.

📦 Repository Structure
wi_sense_rx/: Arduino firmware for the Receiver (RX) ESP32 node.

wi_sense_tx/: Arduino firmware for the Transmitter (TX) ESP32 node.

plot_my_csi.py: Main Python script handling real-time data processing, filtering, and dashboard visualization.

stream_to_ruview.py: Pipeline script for data streaming and classification management.

run_commands.txt: A reference sheet containing pre-configured terminal execution commands.

live_capture.csi.jsonl: Sample JSON Lines log file containing raw captured CSI data packets.

🛠️ Hardware Setup & Deployment
The system requires two ESP32 development boards acting as a transmitter-receiver pair to monitor the environment.

1. Flashing the Hardware
Connect your first ESP32 board to your computer and open the Arduino IDE.

Flash the firmware located inside the wi_sense_rx/ folder to this board. This will act as your dedicated Receiver (RX).

Connect your second ESP32 board and flash the firmware from the wi_sense_tx/ folder. This will be your Transmitter (TX).

2. Node Positioning
Receiver (RX): Must remain connected to your host PC via USB to stream the captured CSI data packet stream to the Python backend.

Transmitter (TX): Once flashed, it operates completely standalone as a mobile node. You can keep it connected to the PC or disconnect it and power it via an external power bank or wall adapter. Place it across the room/area you want to monitor.

🚀 Running the System
Once the hardware is powered on and the RX node is connected via serial/USB:

Open your terminal or command prompt inside the project root directory.

Refer to run_commands.txt to copy and execute the appropriate initialization commands.

Run the primary ingestion script to launch the live processing pipeline:

Bash
python plot_my_csi.py

📊 Understanding the Dashboard
When the pipeline initializes, a real-time matplotlib dashboard will open consisting of three main analysis modules:

1. CSI Amplitude-Deviation Heatmap (Top)
Displays the shifting patterns of active subcarriers over time (Packet Index).

Static environment: Solid, consistent color bands.

Physical movement: Drastic vertical color shifts representing phase and amplitude distortion across subcarriers.

2. Motion Energy Pipeline (Middle)
Tracks raw signal alterations passing through a Savitzky-Golay smoothing filter compared against an Adaptive Threshold line.

IDLE / STATIC (Green): Signal energy remains safely below the adaptive threshold.

MOTION DETECTED (Pink Bars): When spatial movement breaks the baseline threshold, the system flags the exact packets with bright pink vertical indicator bars, returning automatically back to an idle state once movement ceases.

3. Per-Subcarrier Amplitude Variance (Bottom)
A real-time histogram monitoring which specific subcarriers are experiencing the highest level of variance, helping isolate static environmental noise from actual physical displacement.

📋 Requirements
Arduino IDE (with ESP32 board manager installed)

Python 3.x

Required Python dependencies (e.g., matplotlib, numpy, scipy)