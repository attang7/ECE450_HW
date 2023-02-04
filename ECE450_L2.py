#----------
#Authors: Anthony Tang, Bao Phan
#ECE 450 Assignment Lab 2
#Due Date: September 26, 2022
#----------
#Title: Dead Reckoning Navigation
#Description: Test robot mobility via photo interrupters and wheel encoders.
#----------

import RPi.GPIO as GPIO
import time

class AlphaBot(object):
	
class AlphaBot(object):
	
	def __init__(self,in1=12,in2=13,ena=6,in3=20,in4=21,enb=26):
		self.IN1 = in1 #in1 = GPIO BCM pin 12 = Left Motor Forward
		self.IN2 = in2 #in2 = GPIO BCM pin 13 = Left Motor Backward
		self.IN3 = in3 #in3 = GPIO BCM pin 20 = Right Motor Backward
		self.IN4 = in4 #in4 = GPIO BCM pin 21 = Right Motor Forward
		self.ENA = ena #ena = GPIO BCM pin 6 = Enable Motor L
		self.ENB = enb #enb = GPIO BCM pin 26 = Enable Motor R

		GPIO.setmode(GPIO.BCM)  #setting mode to BCM pin names
		GPIO.setwarnings(False) #setting warnings to false to ignore them
		GPIO.setup(self.IN1,GPIO.OUT) #setup the pins for motor control
		GPIO.setup(self.IN2,GPIO.OUT)
		GPIO.setup(self.IN3,GPIO.OUT)
		GPIO.setup(self.IN4,GPIO.OUT)
		GPIO.setup(self.ENA,GPIO.OUT)
		GPIO.setup(self.ENB,GPIO.OUT)
		self.forward()
		self.PWMA = GPIO.PWM(self.ENA,500) #set PWM of motors to freq of 500 Hz
		self.PWMB = GPIO.PWM(self.ENB,500)
		self.PWMA.start(50) #start PWM of motors at 50% duty cycle
		self.PWMB.start(50)

	def forward(self):
		GPIO.output(self.IN1,GPIO.HIGH) #Left Forward
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH) #Right Forward

	def stop(self):
		# GPIO.output(self.IN1,GPIO.LOW)
		# GPIO.output(self.IN2,GPIO.LOW)
		# GPIO.output(self.IN3,GPIO.LOW)
		# GPIO.output(self.IN4,GPIO.LOW)

	def backward(self):
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.HIGH) #Left Backward
		GPIO.output(self.IN3,GPIO.HIGH) #Right Backward
		GPIO.output(self.IN4,GPIO.LOW)

	def left(self):
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.HIGH) #Left Backward
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH) #RIght Forward

	def right(self):
		GPIO.output(self.IN1,GPIO.HIGH) #Left Forward
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.HIGH) #Right Backward
		GPIO.output(self.IN4,GPIO.LOW)
	
    #code to change PWM of wheel
	def setPWMA(self,value):
		self.PWMA.ChangeDutyCycle(value)

	def setPWMB(self,value):
		self.PWMB.ChangeDutyCycle(value)
          


#global variables
motorL = 7 #SPI CE0 pin = Photo interrupter
motorR = 8 #SPI CE1 pin = Photo interrupter
GPIO.setmode(GPIO.BCM) #setting mode to BCM pin names
GPIO.setwarnings(False) #setting warnings to false to ignore them
GPIO.setup(motorL, GPIO.IN, pull_up_down = GPIO.PUD_DOWN) #setup photo interupters
GPIO.setup(motorR, GPIO.IN, pull_up_down = GPIO.PUD_DOWN) #added "pull_up_down = GPIO.PUD_DOWN" from
                                                          #troubleshooting a prior error

#track left and right photo interrupts
trackL = 0
trackR = 0

#functions to update trackL and trackR when event detection triggers
#format referenced from Prof. Schaefer slide "Wheel Encoder Event Detection (with callback function)"
def updateTrackL(ev=None):
    global trackL
    trackL += 1
    print("L = ",trackL)
    
def updateTrackR(ev=None):
    global trackR
    trackR += 1
    print("R = ",trackR)
    
#event detection of photo interrupters, callback to update functions to increment trackers
GPIO.add_event_detect(motorL, GPIO.BOTH, callback = updateTrackL)
GPIO.add_event_detect(motorR, GPIO.BOTH, callback = updateTrackR)

#our PWM values for our crappy robot wheels
speedL = 26
speedR = 23

car = AlphaBot()

car.setPWMA(speedL)
car.setPWMB(speedR)

# ----------7 feet--------------------------
while (trackL <= 150): #monitoring trackL tacks til desired distance
	car.forward() #move AlphaBot forward while in loop
car.stop() #stop AlphaBot
time.sleep(3) #pasue 3 seconds

trackL = 0 #reset trackers
trackR = 0

# turn 360 degree is around 40 ticks
while (trackL <= 41):
	car.right()	
car.stop()
time.sleep(3)

trackL = 0
trackR = 0
while (trackL <= 140):
	car.forward()
car.stop()
time.sleep(3)
#------------------------------------------

# turn right 90 degree is around 10 ticks
trackL = 0
trackR = 0
while (trackL <= 10):
	car.right()	
car.stop()
time.sleep(3)
#-----------------------------------------
	
# straight 3 feet
trackL = 0
trackR = 0
while (trackL <= 130 ):
	car.forward()	
car.stop()
time.sleep(1)	
#-----------------------------------------

# turn right 90 degree
trackL = 0
trackR = 0
while (trackL <= 9):
	car.right()	
car.stop()
time.sleep(3)
	
# straight 7 feet
trackL = 0
trackR = 0
while (trackL <= 290):
	car.forward()	
car.stop()
time.sleep(1)

# turn right 90 degree
trackL = 0
trackR = 0
while (trackL <= 10):
	car.right()	
car.stop()
time.sleep(3)
	
# straight 2 feet
trackL = 0
trackR = 0
while (trackL <= 80):
	car.forward()	
car.stop()
time.sleep(1)		
		
# turn right 90 degree
trackL = 0
trackR = 0
while (trackL <= 10):
	car.right()	
car.stop()
time.sleep(3)
	
# straight 5 feet
trackL = 0
trackR = 0
while (trackL <= 210):
	car.forward()	
car.stop()
time.sleep(1)

# turn right 90 degree
trackL = 0
trackR = 0
while (trackL <= 10):
	car.right()	
car.stop()
time.sleep(3)
	
# straight 1 feet
trackL = 0
trackR = 0
while (trackL <= 50):
	car.forward()	
car.stop()
time.sleep(1)
	

GPIO.cleanup()
