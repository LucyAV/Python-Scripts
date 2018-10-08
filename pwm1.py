import wiringpi
from time import sleep

# 117 to 162 are possible values for steering servo - 118 to 161 are safer)

numClock = 192;
numRange = 2000;

### Initialize WiringPi, then Pin, then PWM Mode
##wiringpi.wiringPiSetupGpio()
##wiringpi.pinMode(18, 2)
##wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
##
### Configure Frequency
##wiringpi.pwmSetClock(numClock)
##wiringpi.pwmSetRange(numRange)
##
### Set Duty Cycle
##wiringpi.pwmWrite(18, 140)


# Initialize WiringPi, then Pin, then PWM Mode
wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(19, 2)
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

# Configure Frequency
wiringpi.pwmSetClock(numClock)
wiringpi.pwmSetRange(numRange)

# Set Duty Cycle
print("Stop for 1 s")
wiringpi.pwmWrite(19, 150)
sleep(1)

wiringpi.pwmWrite(19, 174)
sleep(1.5)

wiringpi.pwmWrite(19, 130)
sleep(0.6)

print("Stop")
wiringpi.pwmWrite(19, 150)
print("Done")

# 158 enough for going forward freely and on floor
# 138 enough for going forward freely and on floor
# both directions might need kickstart

# 175 is speed max
