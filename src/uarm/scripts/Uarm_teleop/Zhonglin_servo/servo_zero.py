import serial
import time 
import numpy as np
import re

NUM_SERVOS = 8
START_SERVO_ID = 1

init_qpos = np.array([14.1, -8, -24.7, 196.9, 62.3, -8.8, 0.0, 0.0])
init_qpos = np.radians(init_qpos)


# Set serial port parameters
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200

def send_command(ser, cmd):
    ser.write(cmd.encode('ascii'))
    time.sleep(0.01)
    response = ser.read_all()
    # print(f"Raw response (binary): {response}")  # Print raw binary response
    return response.decode('ascii', errors='ignore')

def pwm_to_angle(response_str, pwm_min=500, pwm_max=2500, angle_range=270):
    match = re.search(r'P(\d{4})', response_str)
    if not match:
        return None
    pwm_val = int(match.group(1))
    pwm_span = pwm_max - pwm_min
    angle = (pwm_val - pwm_min) / pwm_span * angle_range
    return angle

def angle_to_gripper(angle_deg, angle_range=270, pos_min=50, pos_max=730):
    """
    Map servo angle (degrees) to gripper position.

    Parameters:
    - angle_deg: Servo angle in degrees
    - angle_range: Maximum servo angle (default 270°)
    - pos_min: Gripper closed position (default 50)
    - pos_max: Gripper open position (default 730)

    Returns:
    - gripper position (integer)
    """
    ratio = (angle_deg / angle_range) * 3
    position = pos_min + (pos_max - pos_min) * ratio
    return int(np.clip(position, pos_min, pos_max))

def main():
    arm_pos = [0.0] * NUM_SERVOS
    angle_pos = [0.0] * NUM_SERVOS
    zero_angles = [0.0] * NUM_SERVOS
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01) as ser:
        print("Serial port opened")

        version_response = send_command(ser, '#000PVER!')
        
        for servo_id in range(START_SERVO_ID, START_SERVO_ID + NUM_SERVOS):
            cmd = f'#{servo_id:03d}PULK!'
            response = send_command(ser, cmd)
            print(f"Servo {servo_id} torque released: {response.strip()}")

        print(f"Version response: {version_response.strip()}")
        while True:
            for servo_id in range(START_SERVO_ID, START_SERVO_ID + NUM_SERVOS):
                index = servo_id - START_SERVO_ID
                cmd = f'#{servo_id:03d}PRAD!'
                response = send_command(ser, cmd)
                angle = pwm_to_angle(response.strip())
                angle_pos[index] = angle
                if angle is not None:
                    angle_offset = angle - zero_angles[index] + init_qpos[index]
                    angle_rad = np.radians(angle_offset)
                    arm_pos[index] = angle_rad
            formatted_angle_pos = ", ".join(
                "None" if angle is None else f"{angle:.3f}" for angle in angle_pos
            )
            print(f"[{formatted_angle_pos}]")
    
if __name__ == "__main__":
    main()
