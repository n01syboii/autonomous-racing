import os
import ydlidar
import numpy as np
import time
from Rosmaster_Lib import Rosmaster

bot = Rosmaster()

RMAX = 200.0

ports = ydlidar.lidarPortList()
port = "/dev/ydlidar"
for key, value in ports.items():
    port = value

laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 512000)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 20)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
laser.setlidaropt(ydlidar.LidarPropMaxRange, 200.0)
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.01)
scan = ydlidar.LaserScan()

# PID Parameters for Steering Control
Kp = 0.61   # Proportional Gain - Adjusts reaction to angle deviations
Ki = 0.0  # Integral Gain - Corrects accumulated small errors
Kd = 1.18   # Derivative Gain - Damps oscillations

prev_error = 0
integral = 0

def vector_addition_with_normalization(scan):
    x_sum = 0
    y_sum = 0

    for point in scan.points:
        angle_degrees = int(np.degrees(point.angle) % 360)  # Convert angle to degrees

        # Filter angles in the range 135° to 225°
        if 105 <= angle_degrees <= 255:
            adjusted_angle = angle_degrees - 180  # Shift reference to 180° as origin
            angle_radians = np.radians(adjusted_angle)

            # Convert to Cartesian coordinates and sum for vector addition
            x_sum += point.range * np.cos(angle_radians)
            y_sum += point.range * np.sin(angle_radians)

    # Compute resultant vector
    resultant_magnitude = np.sqrt(x_sum**2 + y_sum**2)
    resultant_angle = np.degrees(np.arctan2(y_sum, x_sum))

    # Normalize resultant angle to the range 0° to 180°
    if -90 <= resultant_angle <= 90:
        normalized_angle = ((resultant_angle + 45) / 90) * 180
    else:
        normalized_angle = None  # Outside the expected range

    return resultant_magnitude, resultant_angle, normalized_angle

def pid_steering(target_angle, current_angle):
    global prev_error, integral

    error = target_angle - current_angle
    integral += error  # Sum of past errors (integral term)
    derivative = error - prev_error  # Change in error (derivative term)

    output = -(Kp * error + Ki * integral + Kd * derivative)  # PID equation

    # Clamp PID output within a reasonable range
    max_steer_adjust =90  # Maximum steering adjustment
    output = max(-max_steer_adjust, min(max_steer_adjust, output))

    prev_error = error
    return output

def animate():
    r = laser.doProcessSimple(scan)
    if r:
        resultant_magnitude, resultant_angle, normalized_angle = vector_addition_with_normalization(scan)

        if normalized_angle is not None:
            print(f"Resultant Vector: Magnitude = {resultant_magnitude:.2f}, Original Angle = {resultant_angle:.2f}°")
            print(f"Normalized Angle (0° to 180°): {normalized_angle:.2f}°")

            # Apply PID for Steering
            pid_output = pid_steering(90, normalized_angle)  
            steering_angle = 90 + pid_output  # Adjust servo angle based on PID output
            steering_angle = max(0, min(180, steering_angle))  # Clamp to 0° - 180°

            print(f"Steering Correction: {pid_output:.2f}, New Steering Angle: {steering_angle:.2f}°")

            # Keep speed constant
            bot.set_motor(0, 100, 0, 100)  
            bot.set_pwm_servo(1, steering_angle)  # Apply PID-adjusted steering
        else:
            print("Normalized Angle: Out of range (-45° to 45°)")
            
        print("Steering output", pid_output)
        
ret = laser.initialize()
if ret:
    ret = laser.turnOn()
    while True:
        animate()
  
laser.turnOff()
laser.disconnecting()