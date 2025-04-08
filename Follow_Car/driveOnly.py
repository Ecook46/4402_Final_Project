# import necessary packages
import time
import RPi.GPIO as GPIO
from motor import *
from servo import *
from rpi_ws281x import *
import traceback

def drive():
    # setup necessary control objects
    ultra = Ultrasonic()
    infrared=Line_Tracking()
    driver = Drive()

    try:
        while True:
            x = ultra.collisionDetect() # detect and adjust speeds
            driver.straight(x) # drive

    # if an exception occurs stop the car
    except Exception:
        PWM = Ordinary_Car()
        PWM.set_motor_model(0,0,0,0)
        traceback.print_exc()
    except KeyboardInterrupt:
        PWM = Ordinary_Car()
        PWM.set_motor_model(0,0,0,0)
        traceback.print_exc()

class Ultrasonic: # class that manages the ultrasonic sensor, most code from Freenove repository except collisionDetect
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
    
    def collisionDetect(self): # function defined by us and not from Freenove repository
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
            elif closure < 0:  # Moving away
                self.x = min(2, self.x + 0.2)

        # Emergency Stop with low TTC
        if TTC < 0.4:
            self.x = 0

        # emergency stop with low clearance
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

        return self.x # return speed multiplier

    
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
class Drive: # class with numerous driving controls
    def __init__(self):
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

ultra=Ultrasonic()
# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    drive()

