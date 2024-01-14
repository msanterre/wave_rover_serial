import json
import time
import serial

DEFAULT_BAUD_RATE = 1000000
DEFAULT_TIMEOUT = 1

class Robot:
    def __init__(self, port, baud_rate=DEFAULT_BAUD_RATE, timeout=DEFAULT_TIMEOUT):
        """
        Initialize the Robot instance.

        Parameters:
        port (str): The serial port to connect to.
        baud_rate (int): The baud rate for serial communication.
        timeout (float): The timeout in seconds for the serial connection.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_connection = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    def connect(self):
        """
        Establishes a serial connection to the robot.
        """
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            print(f"Connected to {self.port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")

    def disconnect(self):
        """
        Closes the serial connection.
        """
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Serial connection closed.")

    def send_command(self, command):
        """
        Sends a JSON command to the robot via the serial connection.
        """
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(json.dumps(command).encode('utf-8'))
        else:
            print("Serial connection not established.")

    def read_json(self, timeout=1.0):
        """
        Reads data from the serial port until the '}' character is encountered or until timeout.
        Parameters:
        timeout (float): Maximum time to wait for data in seconds.
        Returns:
        str: The data read from the serial port.
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            print("Serial connection not established.")
            return ""

        start_time = time.time()
        read_data = ""
        while True:
            if time.time() - start_time > timeout:
                print("Read timeout.")
                break

            if self.serial_connection.in_waiting > 0:
                char = self.serial_connection.read().decode('utf-8')
                read_data += char
                if char == '}':
                    break

        return read_data

    def readline(self, stop_char='\n', timeout=1.0):
        """
        Reads data from the serial port until the stop character is encountered or until timeout.
        Parameters:
        stop_char (str): Character to stop reading at.
        timeout (float): Maximum time to wait for data in seconds.
        Returns:
        str: The data read from the serial port.
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            print("Serial connection not established.")
            return ""

        start_time = time.time()
        read_data = ""
        while True:
            if time.time() - start_time > timeout:
                print("Read timeout.")
                break

            if self.serial_connection.in_waiting > 0:
                char = self.serial_connection.read().decode('utf-8')
                read_data += char
                if char == stop_char:
                    break

        return read_data

    def emergency_stop(self):
        """
        Triggers an emergency stop. This command is used to immediately halt all robot actions.
        """
        command = {"T": 0}
        self.send_command(command)

    def speed_input(self, left_speed=255, right_speed=255):
        """
        Controls the robot's movement speed.
        Parameters:
        left_speed (int): Power for the left side motor (range -255 to 255).
        right_speed (int): Power for the right side motor (range -255 to 255).
        """
        command = {"T": 1, "L": left_speed, "R": right_speed}
        self.send_command(command)

    def pid_set(self, p=170, i=90):
        """
        Sets PID parameters for closed-loop motor control.
        Note: Not available for chassis without speed feedback.
        Parameters:
        p (int): Proportional parameter.
        i (int): Integral parameter.
        """
        command = {"T": 2, "P": p, "I": i}
        self.send_command(command)

    def oled_set(self, line_num, text):
        """
        Sets the content of the OLED display.
        Parameters:
        line_num (int): The line number to set (0-3).
        text (str): The text content to display on the line.
        """
        command = {"T": 3, "lineNum": line_num, "Text": text}
        self.send_command(command)

    def pwm_servo_ctrl(self, pos=90, spd=30):
        """
        Controls the angle of a PWM servo.
        Parameters:
        pos (int): The angle to rotate to (0-180 degrees).
        spd (int): The speed of rotation (not used in the demo).
        """
        command = {"T": 40, "pos": pos, "spd": spd}
        self.send_command(command)

    def pwm_servo_mid(self):
        """
        Sets the PWM servo to its center position (90 degrees).
        """
        command = {"T": -4}
        self.send_command(command)

    def bus_servo_ctrl(self, id, pos, spd, acc):
        """
        Controls a serial bus servo.
        Parameters:
        id (int): ID of the bus servo.
        pos (int): Target position for rotation (0-4095 for the ST3215 servo).
        spd (int): Rotation speed (steps per second).
        acc (int): Acceleration of servo rotation (0-254).
        """
        command = {"T": 50, "id": id, "pos": pos, "spd": spd, "acc": acc}
        self.send_command(command)

    def bus_servo_mid(self, id):
        """
        Rotates the serial bus servo to its middle position.
        Parameters:
        id (int): ID of the target servo.
        """
        command = {"T": -5, "id": id}
        self.send_command(command)

    def bus_servo_scan(self, num=20):
        """
        Scans for connected servos on the bus.
        Parameters:
        num (int): Maximum ID of the servo to scan.
        """
        command = {"T": 52, "num": num}
        self.send_command(command)
        return self.read_json()

    def bus_servo_info(self, id):
        """
        Gets information about a specific servo.
        Parameters:
        id (int): ID of the servo.
        """
        command = {"T": 53, "id": id}
        self.send_command(command)
        return self.read_json()

    def bus_servo_id_set(self, old_id, new_id):
        """
        Changes the ID of a servo.
        Parameters:
        old_id (int): Current ID of the servo.
        new_id (int): New ID to be set for the servo.
        """
        command = {"T": 54, "old": old_id, "new": new_id}
        self.send_command(command)

    def bus_servo_torque_lock(self, id, status):
        """
        Controls the servo torque lock.
        Parameters:
        id (int): ID of the servo.
        status (int): Torque lock status, 1 to enable, 0 to disable.
        """
        command = {"T": 55, "id": id, "status": status}
        self.send_command(command)

    def bus_servo_torque_limit(self, id, limit):
        """
        Sets the servo torque limit.
        Parameters:
        id (int): ID of the target servo.
        limit (int): Torque limiting ratio (500 for 50%, 1000 for 100%).
        """
        command = {"T": 56, "id": id, "limit": limit}
        self.send_command(command)

    def bus_servo_mode(self, id, mode):
        """
        Sets the operation mode of the servo.
        Parameters:
        id (int): ID of target servo.
        mode (int): Operation mode value (0 for position mode, 3 for stepper mode).
        """
        command = {"T": 57, "id": id, "mode": mode}
        self.send_command(command)

    def wifi_scan(self):
        """
        Scans for surrounding WIFI hotspots. Disconnects existing WIFI connection to perform the scan.
        """
        command = {"T": 60}
        self.send_command(command)

    def wifi_try_sta(self):
        """
        Connects to a known WIFI in STA mode.
        """
        command = {"T": 61}
        self.send_command(command)

    def wifi_ap_default(self):
        """
        Enables the WIFI hotspot in AP mode with default settings.
        """
        command = {"T": 62}
        self.send_command(command)

    def wifi_info(self):
        """
        Obtains WIFI information.
        """
        command = {"T": 65}
        self.send_command(command)
        return self.read_json()

    def wifi_off(self):
        """
        Disables the WiFi function.
        """
        command = {"T": 66}
        self.send_command(command)

    def ina219_info(self):
        """
        Gets information about the INA219, including the voltage and current power of the power supply.
        """
        command = {"T": 70}
        self.send_command(command)
        return self.read_json()

    def imu_info(self):
        """
        Obtains IMU information, including heading angle, geomagnetic field, acceleration, attitude, temperature, etc.
        """
        command = {"T": 71}
        self.send_command(command)
        return self.read_json()

    def encoder_info(self):
        """
        Gets motor encoder information. Not applicable to WAVE ROVER as there are no encoders.
        """
        command = {"T": 73}
        self.send_command(command)
        return self.read_json()

    def device_info(self):
        """
        Gets the device information. The information must be customized by the user.
        """
        command = {"T": 74}
        self.send_command(command)
        return self.read_json()

    def io_ir_cut(self, status):
        """
        Controls the IR cut function via the IO5 pin on the driver board.
        Parameters:
        status (int): 1 to enable, 0 to disable the IR cut function.
        """
        command = {"T": 80, "status": status}
        self.send_command(command)

    def set_spd_rate(self, left=1.0, right=1.0):
        """
        Adjusts the power coefficient of each side's motor.
        Parameters:
        left (float): Power coefficient of the left side motor.
        right (float): Power coefficient of the right side motor.
        """
        command = {"T": 901, "L": left, "R": right}
        self.send_command(command)

    def get_spd_rate(self):
        """
        Retrieves the current power coefficients of the left and right side motors.
        """
        command = {"T": 902}
        self.send_command(command)
        return self.read_json()

    def spd_rate_save(self):
        """
        Saves the current motor speed rate settings in the ESP32's non-volatile storage (NVS).
        """
        command = {"T": 903}
        self.send_command(command)

    def get_nvs_space(self):
        """
        Retrieves the remaining space in the NVS area of the ESP32.
        """
        command = {"T": 904}
        self.send_command(command)
        return self.read_json()

    def nvs_clear(self):
        """
        Clears the NVS area of the ESP32, resetting stored values to their default.
        """
        command = {"T": 905}
        self.send_command(command)
