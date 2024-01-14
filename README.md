# Wave Rover Serial

`wave_rover_serial` is a Python package designed for controlling and interacting with the Wave Rover robotic systems via serial communication. It offers a comprehensive set of functionalities to command various aspects of the robot, including movement, servo control, sensor data acquisition, and wireless communication management.

## References

https://www.waveshare.com/wiki/WAVE_ROVER

## Features

- Control motor speeds, servo angles, and other robotic components.
- Read sensor data and system information from the Wave Rover.
- Manage wireless connections, including WiFi scanning and connection.
- Simplified interface for robust serial communication with the robot.

## Installation

To install `wave_rover_serial`, use the following pip command:

```bash
pip install wave_rover_serial
```

## Dependencies

- pyserial: This is required for handling serial communication with the robot.

## Usage

Here's a quick example of how to use the wave_rover_serial package:

```python
from wave_rover_serial import Robot

# Initialize the Robot with the appropriate serial port and baud rate
robot = Robot('/dev/ttyUSB0')

# Connect to the robot
robot.connect()

# Send a command to set motor speeds
robot.speed_input(left_speed=100, right_speed=100)

# Send a command to get the IMU information and read the response
data = robot.imu_info()
print(data)

# Safely disconnect when done
robot.disconnect()
```

You can also use the `with` statement to automatically connect and disconnect from the robot:

```python
from wave_rover_serial import Robot

# Initialize the Robot with the appropriate serial port and baud rate
with Robot('/dev/ttyUSB0') as robot:
    # Send a command to set motor speeds
    robot.speed_input(left_speed=100, right_speed=100)

    # Send a command to get the IMU information and read the response
    data = robot.imu_info()
    print(data)
```

## Documentation

Each method within the Robot class is designed to interact with specific functionalities of the Wave Rover robotic system. Detailed descriptions and usage instructions for each method can be found in the module's documentation.

## Contributing

We welcome contributions to the wave_rover_serial package. Feel free to submit pull requests, report bugs, or propose new features.

## License

wave_rover_serial is distributed under the MIT License.

## Issues

For inquiries or support regarding wave_rover_serial, please file an issue on this repo.
