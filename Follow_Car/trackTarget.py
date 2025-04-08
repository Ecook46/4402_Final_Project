# import necessary packages
import cv2
from ultralytics import YOLO
from picamera2 import Picamera2
import time
from servo import *

def look():
    try:
        # instantiate model and components and initial parameters
        servo = Servo()
        cam = Picamera2()
        model = YOLO("targetTrack.pt", task="detect")
        initial_servo_position = 90  # 90 degrees is the neutral position
        current_x_position = 90
        current_y_position = 90

        # set servo to neutral and start camera feed 
        servo.set_servo_pwm('1', initial_servo_position)
        servo.set_servo_pwm('0', initial_servo_position)
        cam.start()

        while True:
            frame = cam.capture_array() # get a new frame
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) # convert from 4 colour channels to 3
            results = model(frame) # run inference on frame

            if results[0]: # if there was a detection
                result = results[0] # extract it
                x_center = result.boxes.xywh.cpu().numpy()[0][0] # get the centroid
                y_center = result.boxes.xywh.cpu().numpy()[0][1]
                print(f"Center (x, y): ({x_center}, {y_center})")

                # Calculate the center of the frame
                frame_center_x = frame.shape[1] / 2
                frame_center_y = frame.shape[0] / 2

                # Calculate the differences between the detected center and the frame center
                x_diff = -(x_center - frame_center_x)
                y_diff = y_center - frame_center_y

                # Adjust servo positions based on x_diff and y_diff
                if abs(x_diff) > 3:  # Only adjust if the object is far from the center
                    new_x_position = initial_servo_position - int(x_diff / frame.shape[1] * 40) # calculate new position
                    new_x_position = max(30, min(150, new_x_position))  # Ensure it stays within 30 to 150 degrees

                    # Adjust in 2-degree increments for smooth transitions
                    while abs(new_x_position - current_x_position) > 2:
                        if new_x_position > current_x_position:
                            current_x_position += 2
                        else:
                            current_x_position -= 2
                        servo.set_servo_pwm('0', current_x_position)
                        time.sleep(0.05)  # Small delay between steps

                    print(f"X adjustment: {new_x_position} degrees")

                if abs(y_diff) > 3:  # Only adjust if the object is far from the center
                    new_y_position = initial_servo_position - int(y_diff / frame.shape[0] * 40)
                    new_y_position = max(30, min(150, new_y_position))  # Ensure it stays within 30 to 150 degrees

                    # Adjust in 2-degree increments for smooth transitions
                    while abs(new_y_position - current_y_position) > 2:
                        if new_y_position > current_y_position:
                            current_y_position += 2
                        else:
                            current_y_position -= 2
                        servo.set_servo_pwm('1', current_y_position)
                        time.sleep(0.05)  # Small delay between steps

                    print(f"Y adjustment: {new_y_position} degrees")

            time.sleep(1)  # Delay to prevent servo jitter

    except KeyboardInterrupt:
        print("Interrupted by user. Closing camera and resetting servos.")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Close the camera and reset servos to neutral
        cam.close()
        servo.set_servo_pwm('1', 90)
        servo.set_servo_pwm('0', 90)
        print("Camera closed and servos reset to 90 degrees.")

    return 0 

look() # run the method
