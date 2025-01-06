import cv2
import numpy as np
from tf_transformations import euler_from_quaternion
import math
import board
import busio
import time
from adafruit_bno055 import BNO055_I2C
import serial
import threading
from gpiozero import PWMOutputDevice

PIN = 17 

motor = PWMOutputDevice(PIN, frequency=50)

# Initialize serial communication
try:
    ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace '/dev/ttyUSB0' with the correct port for your Arduino
    time.sleep(2)  # Wait for the connection to establish
except serial.SerialException as e:
    print(f"Serial error: {e}")
    ser = None
except Exception as e:
    print(f"An error occurred: {e}")
    ser = None

# Initialize IMU
i2c = busio.I2C(board.SCL, board.SDA)
sensor = BNO055_I2C(i2c)

# PID variables
depth = 0
previous_error_roll = 0.0
previous_error_pitch = 0.0
previous_error_depth = 0.0
roll = pitch = yaw = 0.0
target_roll = target_pitch = 0.0

rkp = 5.8
rkd = 15.7
rki = 0.001

pkp = 1.0
pkd = 1.0
pki = 0.001

dkp = 1.0
dkd = 0.0
dki = 0.0

kp = 0.5
kd = 0.0
ki = 0.0

ckp = 1.0
ckd = 0.0
cki = 0.0



duration = previous_time = roll_integral = pitch_integral = depth_derivative = depth_integral = 0.0
motorA = motorB = motorC = 0.0
motor_left = motor_right = 0.0


def PIDThread():
    global roll, pitch, yaw, previous_error_roll, previous_error_pitch, duration, previous_time, roll_integral, pitch_integral
    global motorA, motorB, motorC, depth, previous_error_depth, depth_derivative, depth_integral

    while True:
        imu_data = sensor.quaternion
        if imu_data is not None:
            start_time = time.time()
            duration = start_time - previous_time

            w, x, y, z = imu_data
            orientation_list = [x, y, z, w]
            roll, pitch, yaw = euler_from_quaternion(orientation_list)
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)

            error_roll = target_roll - roll
            error_pitch = target_pitch - pitch
            error_depth = 200 - depth

            roll_derivative = (error_roll - previous_error_roll)
            pitch_derivative = (error_pitch - previous_error_pitch)
            depth_derivative = (error_depth - previous_error_depth)

            roll_integral += error_roll * duration
            pitch_integral += error_pitch * duration
            depth_integral += error_depth * duration

            pid_roll = (rkp * error_roll) + (rkd * roll_derivative) + (rki * roll_integral)
            pid_pitch = (pkp * error_pitch) + (pkd * pitch_derivative) + (pki * pitch_integral)
            pid_depth = (dkp * error_depth) + (dkd * depth_derivative) + (dki * depth_integral)

            previous_error_roll = error_roll
            previous_error_pitch = error_pitch
            previous_error_depth = error_depth

            previous_time = start_time
            motorA = 1500 + float((pid_pitch)) # -  (pid_depth))
            motorB = 1500 + float((pid_roll) - ((1 / 2) * pid_pitch)) # -pid_depth))
            motorC = 1500 + float((-pid_roll) - ((1 / 2) * pid_pitch)) # -pid_depth))

def SerialCommunication():
    """Handles sending and receiving serial data with the Arduino."""
    global motorA, motorB, motorC, motor_right,motor_left

    while True:
        # Prepare throttle string from PID values
        throttle_data = f"{motorB:.2f} {motor_right:.2f} {motorA:.2f} {motor_left:.2f} {motorC:.2f}"
        ser.write((throttle_data + "\n").encode())  # Send throttle values to Arduino
        print(f"Sent throttle values: {throttle_data}")
        time.sleep(0.1)

        

        
def nothing(x):
    pass


        
def detect_orange(frame):
    
    # Define lower and upper bounds for the HSV mask
    l_b = np.array([10, 100, 100])
    u_b = np.array([25, 255, 255])

    # Convert the frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Create a mask for orange color
    orange_mask = cv2.inRange(hsv_frame, l_b, u_b)

    detected_orange = np.any(orange_mask > 0)

    return orange_mask, detected_orange

def detect_red(frame):
    # Define lower and upper bounds for red color in two ranges
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Convert the frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for both ranges of red
    mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)

    # Combine the masks
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Find contours in the red mask
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    is_square_detected = False  # Boolean to track square detection
    object_centers = []  # List to store the centers of detected objects

    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Filter small objects
            # Draw the actual contour on the frame
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)  # Green contour around detected red object

            # Calculate the center of the contour using moments
            M = cv2.moments(contour)
            if M["m00"] != 0:  # Avoid division by zero
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                object_centers.append((cx, cy))  # Append the center coordinates
                # Draw a circle at the center
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)  # Blue dot for the center

            # Approximate the contour to a polygon
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the polygon has 4 sides (potential square or rectangle)
            if len(approx) == 4:
                # Calculate aspect ratio to confirm it's a square
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h

                if 1.0 <= aspect_ratio <= 1.03:  # Aspect ratio close to 1
                    is_square_detected = True

    detected_red = np.any(red_mask > 0)  # Check if any red is detected

    return red_mask, detected_red, is_square_detected, object_centers



def line_follow(mask, frame):
    global motorA, motorB, motorC, motor_left, motor_right
    height, width, _ = frame.shape

    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))

    # Divide the frame into 4 equal horizontal parts
    frame_centers = []
    segment_height = height // 4
    total_distance = 0
    total_comparisons = 0
    previous_error_roll = 0

    for i in range(4):
        y_center = (i * segment_height) + (segment_height // 2)
        x_center = width // 2
        frame_centers.append((x_center, y_center))
        cv2.circle(frame, (x_center, y_center), 5, (255, 0, 0), -1)  # Blue dots for frame centers

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # Process each contour
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Filter small contours
            # Draw the contour
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

            # Get the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Divide the bounding rectangle into 4 equal vertical parts
            contour_centers = []
            segment_height_contour = h // 4
            for j in range(4):
                cx = x + w // 2  # Horizontal center
                cy = y + j * segment_height_contour + segment_height_contour // 2
                contour_centers.append((cx, cy))
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)  # Red dots for contour centers

            # Calculate distances between frame centers and contour centers
            for i, fc in enumerate(frame_centers):
                for j, cc in enumerate(contour_centers):
                    distance = (cc[0] - fc[0]) + (cc[1] - fc[1])
                    total_distance += distance
                    total_comparisons += 1
                    print(f"Distance from Frame Center {i + 1} to Contour Center {j + 1}: {distance:.2f}")

            if total_comparisons > 0:
                average_distance = total_distance / total_comparisons
                print("avg",average_distance)

                roll_derivative = average_distance - previous_error_roll

                pid_roll = (kp * average_distance) + (kd * roll_derivative)

                previous_error_roll = average_distance

                
                motor_left = clamp(1750 + float(pid_roll), 1500, 2000)
                motor_right = clamp(1750 + float(-pid_roll), 1500, 2000)

    # Ensure motor values are within bounds
    motorA = clamp(motorA, 1500, 2000)
    motorB = clamp(motorB, 1500, 2000)
    motorC = clamp(motorC, 1500, 2000)
    motor_left = clamp(motor_left, 1500, 2000)
    motor_right = clamp(motor_right, 1500, 2000)

    # Print and send throttle values
    print("Throttle values from PID:")
    print(f"Motor A: {motorA}, Motor B: {motorB}, Motor C: {motorC}, Motor Left: {motor_left}, Motor Right: {motor_right}")
    
    # Display the frame
    cv2.imshow("Frame", frame)


    
def container_callibration(mask, frame, object_centers):
    global motorA, motorB, motorC, motor_left, motor_right
    avg = 0
    height, width, _ = frame.shape

    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))

    # Define the center of the frame
    frame_center_x = width // 2
    frame_center_y = height // 2
    previous_error_roll = 0

    # Draw blue box at the center of the frame
    cv2.rectangle(frame, (frame_center_x - 10, frame_center_y - 10), 
                  (frame_center_x + 10, frame_center_y + 10), (255, 0, 0), 2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour (assuming it’s the container)
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Initialize default container center
        container_center_x = None
        container_center_y = None

        # Calculate the center of the green box (container)
        if object_centers:  # Ensure object_centers is not empty
            for i, (cx, cy) in enumerate(object_centers):
                container_center_x = cx
                container_center_y = cy

        # Verify the container center is defined
        if container_center_x is not None and container_center_y is not None:
            # Draw a yellow circle at the center of the green box
            cv2.circle(frame, (container_center_x, container_center_y), 5, (0, 255, 255), -1)
            
            # Calculate error
            error_x = container_center_x - frame_center_x
            error_y = container_center_y - frame_center_y
            avg = (error_x + error_y) // 2

            roll_derivative = avg - previous_error_roll
            pid_roll = (ckp * avg) + (ckd * roll_derivative)
            previous_error_roll = avg

            motor_left = clamp(1750 + float(pid_roll), 1500, 2000)
            motor_right = clamp(1750 + float(-pid_roll), 1500, 2000)


    # Ensure motor values are within bounds
    motorA = clamp(motorA, 1500, 2000)
    motorB = clamp(motorB, 1500, 2000)
    motorC = clamp(motorC, 1500, 2000)
    motor_left = clamp(motor_left, 1500, 2000)
    motor_right = clamp(motor_right, 1500, 2000)
    
    if avg < 5:
    
        print("Full throttle")
        motor.value = .075

        
        motor.value = 0  # Stop PWM
        print("GPIO cleaned up.")  

    # Print and send throttle values
    print("Throttle values from PID:")
    print(f"Motor A: {motorA}, Motor B: {motorB}, Motor C: {motorC}, Motor Left: {motor_left}, Motor Right: {motor_right}")

    # Display the frame
    cv2.imshow("Frame", frame)

def main():

    global motor_left,motor_right

    pid_thread = threading.Thread(target=PIDThread, daemon=True)
    ser_thread = threading.Thread(target=SerialCommunication, daemon=True)
    
    pid_thread.start()
    ser_thread.start()
    
    # Open the webcam
    cap = cv2.VideoCapture(0)
    
    # Create a window for trackbars
    

    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame. Exiting...")
            break
            
         

        orange_mask, is_orange_detected = detect_orange(frame)
        red_mask, is_red_detected, is_square, object_centers = detect_red(frame)
        # Conditional logic for detecting colors
        if is_orange_detected and is_red_detected:
            print("Square detected")
            container_callibration(red_mask, frame, object_centers)
        elif is_red_detected:
            print("square detected.")
            container_callibration(red_mask, frame, object_centers)
        elif is_orange_detected and not is_red_detected:
            print("Orange detected.")
            line_follow(orange_mask, frame)
        elif not is_orange_detected or not is_red_detected:
            motor_left = 1500
            motor_right = 1500
            cv2.imshow("Frame",frame)
            
           


        # Break the loop on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the resources
    cap.release()
    cv2.destroyAllWindows()





if __name__ == "__main__":

    main()
