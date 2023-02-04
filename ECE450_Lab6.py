#----------
#Authors: Anthony Tang, Bao Phan
#ECE 450 Assignment Lab 6
#Due Date: November 28, 2022
#----------
#Title: You are the Professor
#Description: Create a lab with set instructions using what we've learned this semester.
#----------

import RPi.GPIO as GPIO
import time
import random
import picamera
import picamera.array
import numpy as np
	
CS = 5
Clock = 25
Address = 24
DataOut = 23

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
		#self.forward()
		self.PWMA = GPIO.PWM(self.ENA,500) #set PWM of motors to freq of 500 Hz
		self.PWMB = GPIO.PWM(self.ENB,500)
		self.PWMA.start(0) #start PWM of motors at 50% duty cycle
		self.PWMB.start(0)

	def backward(self):
		GPIO.output(self.IN1,GPIO.HIGH) #Left Forward
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH) #Right Forward

	def stop(self):
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.LOW)

	def forward(self):
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
        
class TRSensor(object):
	def __init__(self,numSensors = 5):
		self.numSensors = numSensors
		self.calibratedMin = [0] * self.numSensors
		self.calibratedMax = [1023] * self.numSensors
		self.last_value = 0

	def AnalogRead(self):
		value = [0,0,0,0,0,0]
		#Read Channel0~channel4 AD value
		for j in range(0,6):
			GPIO.output(CS, GPIO.LOW)
			for i in range(0,4):
				#sent 4-bit Address
				if(((j) >> (3 - i)) & 0x01):
					GPIO.output(Address,GPIO.HIGH)
				else:
					GPIO.output(Address,GPIO.LOW)
				#read MSB 4-bit data
				value[j] <<= 1
				if(GPIO.input(DataOut)):
					value[j] |= 0x01
				GPIO.output(Clock,GPIO.HIGH)
				GPIO.output(Clock,GPIO.LOW)
			for i in range(0,6):
				#read LSB 8-bit data
				value[j] <<= 1
				if(GPIO.input(DataOut)):
					value[j] |= 0x01
				GPIO.output(Clock,GPIO.HIGH)
				GPIO.output(Clock,GPIO.LOW)
			#no mean ,just delay
			for i in range(0,6):
				GPIO.output(Clock,GPIO.HIGH)
				GPIO.output(Clock,GPIO.LOW)
#			time.sleep(0.0001)
			GPIO.output(CS,GPIO.HIGH)
		return value[1:]

        
        
#initializing GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Clock,GPIO.OUT)
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)

#setting easier names for later calls
car = AlphaBot()

#V = the value we determien that seperates white from black for track sensors (white > V, Black < V)
V = 950

#our PWM values for our crappy robot wheels
speedL = 28
speedR = 26
car.setPWMA(speedL)
car.setPWMB(speedR)
car.stop()

while(1):
		track = TRSensor().AnalogRead() #read values from IR sensors and print for observation during runs
		print(track[0],track[1],track[2],track[3],track[4])
		with picamera.PiCamera() as camera: #code provided to acquire data from PiCam, specifically light intensity
			camera.resolution = (128, 80)
			with picamera.array.PiRGBArray(camera) as stream:
				camera.exposure_mode = 'auto'
				camera.awb_mode = 'auto'
				camera.capture(stream, format='rgb')
				pixAverage = int(np.average(stream.array[...,1]))
		print ("Light Meter pixAverage=%i" % pixAverage)
    #MOVE FORWARD
    #IR 3 black
		if (pixAverage>=15): #code to check for light intensity, if above set value, then run.
				if(track[2]<V): #nested if statement to check IR sensors to move along track
						print(track[0],track[1],track[2],track[3],track[4],"forward")
						car.forward()
						time.sleep(0.2) #delays put in place to to the delay between the PiCam upating data and the wheels being able ot change direction
						car.stop()
				elif((track[1] or track[0])<V):
						print(track[0],track[1],track[2],track[3],track[4],"left")
						car.left()
						time.sleep(0.1)
						car.stop()
				else:
						print(track[0],track[1],track[2],track[3],track[4],"right")
						car.right()
						time.sleep(0.1)
						car.stop()
		
	#while((track[2] < V)):
	#	track = TRSensor().AnalogRead()
	#	if(pixAverage>=15):
	#		print(track[0],track[1],track[2],track[3],track[4],"forward")
	#		car.forward()
	#	else:
	#		car.stop()
        
        #MOVE RIGHT    
        #IR 4-5 black    
	#while((track[3]) < V):
	#	track = TRSensor().AnalogRead()
	#	if(pixAverage>=15):
	#		print(track[0],track[1],track[2],track[3],track[4],"right")
	#		car.right()
	#	else:
	#		car.stop()
		
        #MOVE LEFT
        #IR 1-2 black    
	#while((track[1]) < V):
	#	track = TRSensor().AnalogRead()
	#	if(pixAverage>=15):
	#		print(track[0],track[1],track[2],track[3],track[4],"left")
	#		car.left()
	#	else:
	#		car.stop()
