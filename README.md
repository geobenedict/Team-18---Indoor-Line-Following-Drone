# Team-18---Indoor-Line-Following-Drone

Welcome to the official repository for **Team 18's Indoor Line Following Drone Project**. This repository contains all the necessary files, including code, design deliverables, bills of materials, 3D print designs, and the design validation tests and results. The aim of this project is to develop a drone capable of autonomously following a predefined path indoors using a line-following navigation system.

## Project Overview

This project focuses on creating a lightweight, agile drone designed for indoor environments, equipped with a line-following system using the **ESP32-CAM** module and **OpenCV** for real-time image processing. The system captures live video, processes it to detect a line, and sends commands to control the drone's movements, ensuring it follows a predefined path.

### Key Features:
- Real-time video streaming via the **ESP32-CAM**.
- Line detection and navigation using **OpenCV**.
- Agile indoor navigation on the **Darwin FPV BabyApe Pro V2** platform.
- Design validation tests for line detection, curve navigation, and T-junction stopping.

## Repository Structure
📂 Team-18-Indoor-Line-Following-Drone ├── 📂 Codes/ # Contains all the code for the ESP32-CAM and OpenCV integration ├── 📂 3d-Designs/ # 3D print designs for drone components ├── 📂 BoM/ # Bill of Materials (BoM) and expenses ├── 📂 Validation-tests/ # Design validation tests and results ├── 📂 Hardware-diagrams/ # Diagrams related to drone hardware and setup ├── 📄 README.md # Project overview and repository guide

## Getting Started

### Prerequisites

- **ESP32-CAM**: Ensure that the ESP32-CAM is set up using the Arduino IDE to act as an Access Point and stream video.
- **OpenCV**: Install   Pycharm to process the video feed and control the drone.
- **Arduino IDE**: Program the ESP32-CAM using the provided `.ino` code files in the `/Codes` folder.

### Installation

1. Clone the repository:
git clone https://github.com/geobenedict/Team-18---Indoor-Line-Following-Drone.git
2. Follow the instructions in the **/Codes** folder to upload the ESP32-CAM code via the Arduino IDE.
3. Use the Python code in the `/Codes` folder to set up the OpenCV environment for processing the video feed.

### Running the Project

1. Connect the **ESP32-CAM** to the drone and power it up.
2. Run the Python script from the `/Codes` folder to process the video stream and control the drone.
3. Validate the drone’s line-following performance by running the tests outlined in the **/Validation-tests** folder.

## Design Deliverables

- **3D Designs**: The `/3d-Designs` folder contains all the 3D print designs for custom drone components.
- **Bills of Materials**: All parts used in the project, along with their costs, are listed in the `/BoM` folder.
- **Hardware Diagrams**: Detailed hardware setup and wiring diagrams are located in the `/hardware-diagrams` folder.
- **Validation Tests and Results**: Test cases for straight-line, curve, and T-junction detection, along with their results, are found in the `/Validation-tests` folder.

## Design Validation

The design validation process included multiple tests, such as:
- **Straight Line Detection Test**: Ensures the drone follows a straight path.
- **Curved Path Detection Test**: Validates the drone’s ability to follow a curved line.
- **T-Junction Detection Test**: Confirms that the drone can detect a T-junction and stop as required.

Test results and detailed analysis are provided in the **/Validation-tests** folder.

**Team 18 - Indoor Line Following Drone Project**
