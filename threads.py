import threading
from time import sleep

import time
import RPi.GPIO as GPIO

TRIG = 23
ECHO = 24

clearPath = False

class ultrasonicThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)

        GPIO.output(TRIG, False)
        sleep(0.1)

        lastDistance = 0
    def run(self):
        
        secondLastDistance = 0
        global clearPath
        
        while True:
            GPIO.output(TRIG, True)
            sleep(0.00001)
            GPIO.output(TRIG, False)

            while GPIO.input(ECHO) == 0:
                pulse_start = time.time()

            while GPIO.input(ECHO) == 1:
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = 17100 * pulse_duration # 17100 better than 17150
            distance = round(distance, 2)
            #print("Distance: ", distance)

            clearPath = False
            if (distance > 100):
                if (lastDistance > 100):
                    if (secondLastDistance > 100):
                        clearPath = True

            secondLastDistance = lastDistance
            lastDistance = distance
            
            sleep(0.1)
        GPIO.cleanup()

# Create and start a new thread
usTh = ultrasonicThread()
usTh.daemon = True
usTh.start()

while True:
    #print("Main Print")
    print(clearPath)
    sleep(0.5)
