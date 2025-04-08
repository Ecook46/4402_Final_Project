import time
import RPi.GPIO as GPIO
from motor import *
from servo import *
from rpi_ws281x import *
import traceback
from picamera2 import Picamera2

def drive(frame_queue, position_queue, exit_flag):
    # setup necessary control objects
    servo = Servo()
    cam = Picamera2()
    ultra = Ultrasonic()
    infrared=Line_Tracking()
    driver = Drive()
    
    last_position = (90,90) # the first position is at neutral

    servo.set_servo_pwm('1', 90) # ensure servos are at neutral
    servo.set_servo_pwm('0', 90)
    cam.start() # start camera feed
    time.sleep(4) # leave time for cross communication

    try:
        while True:
            if not exit_flag.empty(): # if the parent process ended end this process
                raise Exception("Forced termination by parent process")
            
            if not position_queue.empty(): # if new servo instructions were received adjust servo position
                last_position = adjustServo(servo,cam, position_queue, last_position, frame_queue)

            x = ultra.collisionDetect() # run ACC component

            driver.straight(x) # drive

    # end safely
    except Exception:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        PWM = Ordinary_Car()
        PWM.set_motor_model(0,0,0,0) # shutoff motors
        traceback.print_exc() # explain error
        cam.close() # shutoff camera stream
        servo.set_servo_pwm('1', 90) # reset servos to neutral
        servo.set_servo_pwm('0', 90)

def adjustServo(servo, cam, position_queue, last_position, frame_queue):

    new_x_position, new_y_position = position_queue.get() # get new servo instructions from the other process
    current_x_position, current_y_position = last_position # remember current position

    frame = cam.capture_array() # get a new frame
    frame_queue.put(frame) # pass it to the other process

    # adjust the position of the servo smoothly in horizontal plane
    while abs(new_x_position - current_x_position) > 2:
        if new_x_position > current_x_position:
            current_x_position += 2
        else:
            current_x_position -= 2
        servo.set_servo_pwm('0', current_x_position)
        time.sleep(0.01)  # Small delay between steps

    # adjust the position of the servo smoothly in vertical plane
    while abs(new_y_position - current_y_position) > 2:
        if new_y_position > current_y_position:
            current_y_position += 2
        else:
            current_y_position -= 2
        servo.set_servo_pwm('1', current_y_position)
        time.sleep(0.01)  # Small delay between steps

    return (new_x_position, new_y_position) # save current servo position

class Ultrasonic: # class that manages the ultrasonic sensor
    def __init__(self): # initialize ultrasonic sensor object 
        self.x = 2       
        GPIO.setwarnings(False)        
        self.trigger_pin = 27
        self.echo_pin = 22
        self.MAX_DISTANCE = 300               # define the maximum measuring distance, unit: cm
        self.timeOut = self.MAX_DISTANCE*60   # calculate timeout according to the maximum measuring distance
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin,GPIO.OUT)
        GPIO.setup(self.echo_pin,GPIO.IN)
        
    def pulseIn(self,pin,level,timeOut): # obtain pulse time of a pin under timeOut
        t0 = time.time()
        while(GPIO.input(pin) != level):
            if((time.time() - t0) > timeOut*0.000001):
                return 0
        t0 = time.time()
        while(GPIO.input(pin) == level):
            if((time.time() - t0) > timeOut*0.000001):
                return 0
        pulseTime = (time.time() - t0)*1000000
        return pulseTime
    
    def get_distance(self):     # get the measurement results of ultrasonic module,with unit: cm
        distance_cm=[0,0,0,0,0]
        for i in range(5):
            GPIO.output(self.trigger_pin,GPIO.HIGH)      # make trigger_pin output 10us HIGH level 
            time.sleep(0.00001)     # 10us
            GPIO.output(self.trigger_pin,GPIO.LOW) # make trigger_pin output LOW level 
            pingTime = self.pulseIn(self.echo_pin,GPIO.HIGH,self.timeOut)   # read plus time of echo_pin
            distance_cm[i] = pingTime * 340.0 / 2.0 / 10000.0     # calculate distance with sound speed 340m/s
        distance_cm=sorted(distance_cm)
        return  int(distance_cm[2])
    
    def collisionDetect(self):
        threshold = 30  # Distance below which the car must stop
        max_distance_for_adjustment = 150  # Distance below which we adjust x based on relative speed
        count = 0

        # Get a valid first distance reading (filtering out false zeros)
        distance1 = self.get_distance()
        while distance1 == 0:
            distance1 = self.get_distance()

        time.sleep(0.1)  # Wait before next reading

        # Get a valid second distance reading
        distance2 = self.get_distance()
        while distance2 == 0:
            distance2 = self.get_distance()

        # Calculate closure rate (relative speed)
        closure = (distance1 - distance2) / 0.1  

        # Time to collision
        TTC = float('inf') if closure <= 0 else distance2 / closure

        # Adaptive Speed Adjustment 
        if distance2 < max_distance_for_adjustment:
            if closure > 0:  # Getting closer
                self.x = max(1, self.x - 0.2)
            elif closure < 0 and self.x != 0:  # Moving away
                self.x = min(2, self.x + 0.2)

        # Emergency Stop
        if TTC < 0.4:
            self.x = 0

        # Obstacle proximity-based stop 
        if distance2 < threshold:
            while True:
                distance3 = self.get_distance()
                if distance3 != 0 and distance3 < threshold:
                    count += 1
                    if count > 3:  # Ensure the reading is consistent
                        self.x = 0
                        break
                else:
                    count = 0
                    break       # Reset count if we get an invalid reading again
        else:
            # If previously stopped and now distance is safe again
            if self.x == 0:
                self.x = 1

        # Debug prints
        print("Distance2:", distance2)
        print("Closure rate:", closure)
        print("TTC:", TTC)
        print("Throttle x:", self.x)

        return self.x

    
######################################################################################################
class Line_Tracking: # class for managing the line tracking, servo, and general movement
    def __init__(self): # setup infrared pins
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01,GPIO.IN)
        GPIO.setup(self.IR02,GPIO.IN)
        GPIO.setup(self.IR03,GPIO.IN)

    def relativePos(self):
                    # determine line position
        if GPIO.input(self.IR01)==False and GPIO.input(self.IR02)==True and GPIO.input(self.IR03)==False:
            self.LMR= 1 # straight
            self.count = 0
        elif GPIO.input(self.IR01)==True and GPIO.input(self.IR02)==False and GPIO.input(self.IR03)==False:
            self.LMR= 2 # slight left
            self.count = 0
        elif GPIO.input(self.IR01)==False and GPIO.input(self.IR02)==False and GPIO.input(self.IR03)==True:
            self.LMR= 3 # slight right
            self.count = 0
        elif GPIO.input(self.IR01)==True and GPIO.input(self.IR02)==True and GPIO.input(self.IR03)==False:
            self.LMR = 4 # strong left
            self.count = 0
        elif GPIO.input(self.IR01)==False and GPIO.input(self.IR02)==True and GPIO.input(self.IR03)==True:
            self.LMR = 5 # strong right
            self.count = 0
        elif GPIO.input(self.IR01)==False and GPIO.input(self.IR02)==False and GPIO.input(self.IR03)==False and self.LMR == 1:
            self.count = self.count + 1
            if self.count > 5:
                self.LMR = 6 # no path
                self.count = 0
        elif GPIO.input(self.IR01)==True and GPIO.input(self.IR02)==True and GPIO.input(self.IR03)==True:
            self.LMR = 7 # split
            self.count = 0
        elif GPIO.input(self.IR01)==True and GPIO.input(self.IR02)==False and GPIO.input(self.IR03)==True:
            self.LMR = 8 # split
            self.count = 0
        return self.LMR
        
        
######################################################################################################

class Drive:
    def __init__(self): # define control object
        self.PWM = Ordinary_Car()
        return
    
    def straight(self, x): # drive straight
        v = int(800 * x)
        self.PWM.set_motor_model(v,v,v,v)

    def turnR(self, x): # turn slight right
        v1 = int(2500*x)
        v2 = int(-1500*x)
        self.PWM.set_motor_model(v1,v1,v2,v2)
        time.sleep(1)
        self.PWM.set_motor_model(0,0,0,0)

    def turnL(self,x): # turn slight left
        v1 = int(2500*x)
        v2 = int(-1500*x)
        self.PWM.set_motor_model(v2,v2,v1,v1)
        time.sleep(1)
        self.PWM.set_motor_model(0,0,0,0)
######################################################################################################  

