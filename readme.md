# Autonomous Underwater Vehicle (AUV)

This project demonstrates the design and implementation of an Autonomous Underwater Vehicle (AUV) using a Raspberry Pi, an Arduino, and various sensors. The system combines hardware and software components for precise underwater navigation, object detection, and motor control.

---

## **Table of Contents**

1. [Features](#features)
2. [Hardware Requirements](#hardware-requirements)
3. [Software Requirements](#software-requirements)
4. [Setup and Usage](#setup-and-usage)
5. [Usage Scenarios](#usage-scenarios)

---

## **Features**

- **PID Control**: Precise motor movement using a PID control loop.
- **IMU-Based Orientation Tracking**: Real-time orientation measurement using the BNO055 IMU sensor.
- **Color Detection**: Detection of orange and red objects using HSV color masks.
- **Line Following and Object Calibration**: Navigation logic for following lines and detecting calibrated objects.
- **Multithreading**: Efficient handling of multiple tasks (e.g., PID control, object detection) simultaneously.
- **Serial Communication**: Communication with an Arduino for motor control.
- **Modular Design**: Flexibility to add more features or customize hardware setups.

---

## **Hardware Requirements**

1. **Raspberry Pi** (with GPIO support)
2. **IMU Sensor** (BNO055)
3. **PWM-Controlled Motors**
4. **Camera Module** (compatible with OpenCV)
5. **Arduino** (or equivalent microcontroller)
6. **Serial-to-USB Cable**
7. **Additional Peripherals**:
   - LEDs (optional, for debugging)
   - Buzzer (for status feedback)

---

## **Software Requirements**

1. **Python 3.8+**
2. Required Python Libraries:
   - [OpenCV](https://pypi.org/project/opencv-python/) for image processing
   - [NumPy](https://pypi.org/project/numpy/) for numerical computations
   - [gpiozero](https://gpiozero.readthedocs.io/) for GPIO control
   - [Adafruit-BNO055](https://pypi.org/project/adafruit-circuitpython-bno055/) for interfacing with the IMU sensor
   - [pyserial](https://pypi.org/project/pyserial/) for serial communication
   - [tf-transformations](https://pypi.org/project/tf-transformations/) for quaternion to Euler angle conversions
   - `board` and `busio` libraries for I2C communication (installed with Adafruit-BNO055)

### Install Dependencies

Run the following command to install all required dependencies:

```bash
pip install opencv-python numpy gpiozero adafruit-circuitpython-bno055 pyserial tf-transformations
```

---

## **Setup and Usage**

### **Hardware Setup**

1. **IMU Sensor**: Connect the BNO055 IMU sensor to the Raspberry Pi via I2C pins.
2. **Motor Connection**: Connect the motors to the GPIO pins and ensure proper wiring. Use GPIO pin 17 for motor control (modifiable in the code).
3. **Camera Setup**: Attach the camera module and test its functionality using OpenCV.
4. **Arduino**: Connect the Arduino to the Raspberry Pi via a Serial-to-USB cable.
5. **Optional Peripherals**: Connect LEDs and a buzzer for debugging and status feedback.

### **Setup Permissions**

Ensure the Raspberry Pi user has access to the serial port by running:

```bash
sudo usermod -a -G dialout $USER
```

Log out and log back in for the changes to take effect.

### **Run the Code**

1. Clone the repository and navigate to the project folder.
2. Execute the main script:

```bash
python3 main.py
```

### **Calibrate Sensors**

1. **IMU Calibration**: Ensure the BNO055 IMU is properly calibrated.
2. **Color Detection**: Use the color detection functions to verify the HSV ranges for the target environment.

---

## **Usage Scenarios**

1. **Underwater Exploration**: The AUV can navigate underwater environments, avoiding obstacles and detecting specific objects.
2. **Search and Rescue**: The AUV can identify and track targets (e.g., orange or red objects) in emergency scenarios.
3. **Research Applications**: Perform underwater data collection using precise orientation and navigation.

---

## **Notes**

- **PID Calibration**: Adjust `kp`, `ki`, and `kd` values in the code to optimize motor control based on hardware setup.
- **Multithreading**: Ensure your Raspberry Pi can handle multithreading for smooth operation.
- **Hardware Connections**: Double-check all hardware connections before running the script to avoid damage or errors.

---

## **Future Improvements**

1. Add real-time visualization of sensor and navigation data.
2. Implement advanced object detection using deep learning models.
3. Expand functionality for multi-motor and multi-sensor integration.
4. Incorporate additional sensors for depth and temperature measurements.

---

Feel free to explore, modify, and contribute to the project. If you encounter any issues or have suggestions, please open an issue or submit a pull request!

