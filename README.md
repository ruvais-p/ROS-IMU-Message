# ROS IMU Sensor Integration with Adafruit BNO055

This repository contains Arduino code for integrating the Adafruit BNO055 Absolute Orientation Sensor with ROS (Robot Operating System) using the Arduino platform. The code reads sensor data from the BNO055 sensor and publishes it as ROS messages for further processing.

## Prerequisites
- Arduino IDE installed on your system
- ROS installed on your computer
- Adafruit BNO055 Absolute Orientation Sensor
- Microcontroller board compatible with Arduino (e.g., Arduino Uno, Arduino Mega)
- Necessary cables for connecting the sensor to the microcontroller

## Hardware Setup
1. Connect the SCL pin of the BNO055 sensor to the SCL pin of your microcontroller.
2. Connect the SDA pin of the BNO055 sensor to the SDA pin of your microcontroller.
3. Connect the VDD pin of the BNO055 sensor to the 3.3V power supply of your microcontroller.
4. Connect the GROUND pin of the BNO055 sensor to the ground (GND) pin of your microcontroller.

## Software Setup
1. Clone or download this repository to your local machine.
2. Open the Arduino IDE and import the required libraries:
   - `ros.h`
   - `std_msgs/String.h`
   - `Wire.h`
   - `sensor_msgs/Imu.h`
   - `Adafruit_Sensor.h`
   - `Adafruit_BNO055.h`
   - `utility/imumaths.h`
3. Upload the provided code (`ros_imu_integration.ino`) to your Arduino board.

## Usage
1. Ensure that the BNO055 sensor is properly connected to the microcontroller and powered on.
2. Connect the microcontroller to your computer using a USB cable.
3. Open a terminal and run the ROS core:
4. Launch the ROS serial node to communicate with the Arduino:
Replace `/dev/ttyUSB0` with the appropriate serial port of your Arduino.
5. You should now see IMU data being published on the ROS topics.

## Notes
- Make sure to modify the code if you're using a different microcontroller or pin configuration.
- Check the baud rate settings in the Arduino code and match them with the ROS serial node settings.
- Refer to the Adafruit BNO055 datasheet and Arduino documentation for more detailed information about the sensor and its usage.

## Acknowledgments
- This project is based on the Adafruit BNO055 Absolute Orientation Sensor library and the ROS serial communication library for Arduino.
- Thanks to the ROS community for their support and contributions to robotics development.


