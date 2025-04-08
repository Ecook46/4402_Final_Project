# import necessary packages
import time
import cv2
from ultralytics import YOLO

def process_new_frame(frame_queue, position_queue):

    # load the model
    model = YOLO("targetTrack.pt", task="detect")

    # define initial parameters
    initial_servo_position = 90
    position_queue.put((initial_servo_position,initial_servo_position))

    while True: # keep the subprocess alive
        if not frame_queue.empty(): # when a new frame is added

            frame = frame_queue.get() # pop the frame from the queue
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) # convert from 4-channel to 3-channel colours
            results = model(frame) # run the model inference on the frame

            if results[0]: # if there was a detection

                result = results[0] # extract it
                x_center = result.boxes.xywh.cpu().numpy()[0][0] # get the centroid of the target
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

                if abs(y_diff) > 3:  # Only adjust if the object is far from the center
                    new_y_position = initial_servo_position - int(y_diff / frame.shape[0] * 40) # calculate new position
                    new_y_position = max(30, min(150, new_y_position))  # Ensure it stays within 30 to 150 degrees

                # send the desired servo position to the control process
                position_queue.put((new_x_position,new_y_position))

            else: # if no detection reset head to look straight
                position_queue.put((90,90))

        time.sleep(0.1) # frequency of checking for frames

