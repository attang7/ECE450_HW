#----------
#Authors: Anthony Tang, Bao Phan
#ECE 450 Assignment Lab 3
#Due Date: October 11, 2022
#----------
#Title: Line Follower
#Description: Test robot mobility via tracker sensor for line following
#----------
#CODES REFERENCED:
#   AlphaBot.py, provided by professor
#   Infrared_Line_Tracking.py, provided by professor
 

import RPi.GPIO as GPIO
import time

#BCM port numbers for Track Sensor
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
		GPIO.output(self.IN2,GPIO.LOW) #Left Backward
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH) #RIght Forward

	def right(self):
		GPIO.output(self.IN1,GPIO.HIGH) #Left Forward
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW) #Right Backward
		GPIO.output(self.IN4,GPIO.LOW)
	
    #code to change PWM of wheel
	def setPWMA(self,value):
		self.PWMA.ChangeDutyCycle(value)

	def setPWMB(self,value):
		self.PWMB.ChangeDutyCycle(value)
        
class TRSensor(object):
	def __init__(self,numSensors = 5):
		self.numSensors = numSensors #track sensor variables
		self.calibratedMin = [0] * self.numSensors
		self.calibratedMax = [1023] * self.numSensors
		self.last_value = 0

	def AnalogRead(self): #reads light values from track sensor
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
        
        def calibrate(self):
		max_sensor_values = [0]*self.numSensors
		min_sensor_values = [0]*self.numSensors
		for j in range(0,10):
		
			sensor_values = self.AnalogRead();
			
			for i in range(0,self.numSensors):
			
				# set the max we found THIS time
				if((j == 0) or max_sensor_values[i] < sensor_values[i]):
					max_sensor_values[i] = sensor_values[i]

				# set the min we found THIS time
				if((j == 0) or min_sensor_values[i] > sensor_values[i]):
					min_sensor_values[i] = sensor_values[i]

		# record the min and max calibration values
		for i in range(0,self.numSensors):
			if(min_sensor_values[i] > self.calibratedMin[i]):
				self.calibratedMin[i] = min_sensor_values[i]
			if(max_sensor_values[i] < self.calibratedMax[i]):
				self.calibratedMax[i] = max_sensor_values[i]

	def	readCalibrated(self):
		value = 0
		#read the needed values
		sensor_values = self.AnalogRead();

		for i in range (0,self.numSensors):

			denominator = self.calibratedMax[i] - self.calibratedMin[i]

			if(denominator != 0):
				value = (sensor_values[i] - self.calibratedMin[i])* 1000 / denominator
				
			if(value < 0):
				value = 0
			elif(value > 1000):
				value = 1000
				
			sensor_values[i] = value
		
		print("readCalibrated",sensor_values)
		return sensor_values
			
	def readLine(self, white_line = 0):

		sensor_values = self.readCalibrated()
		avg = 0
		sum = 0
		on_line = 0
		for i in range(0,self.numSensors):
			value = sensor_values[i]
			if(white_line):
				value = 1000-value
			# keep track of whether we see the line at all
			if(value > 200):
				on_line = 1
				
			# only average in values that are above a noise threshold
			if(value > 50):
				avg += value * (i * 1000);  # this is for the weighted total,
				sum += value;                  #this is for the denominator 

		if(on_line != 1):
			# If it last read to the left of center, return 0.
			if(self.last_value < (self.numSensors - 1)*1000/2):
				#print("left")
				return 0;
	
			# If it last read to the right of center, return the max.
			else:
				#print("right")
				return (self.numSensors - 1)*1000

		self.last_value = avg/sum
		
		return self.last_value

#initializing GPIO
GPIO.setmode(GPIO.BCM) #setting mode to BCM pin names 
GPIO.setwarnings(False) #setting warnings to false to ignore them
GPIO.setup(Clock,GPIO.OUT) #setting up Clock, Address, CS, and DataOut respectively
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)

#setting easier names for later calls
TR = TRSensor()
car = AlphaBot()

#V = the value we determined that seperates white from black for track sensors in testing (White > V, Black < V)
V = 950

#our PWM values for our crappy robot wheels
speedL = 28
speedR = 23
car.setPWMA(speedL)
car.setPWMB(speedR)
car.forward()

#due to some issues with trying to figure out the infrared line tracker,
#ended up using a simpler ON/OFF code depending on which sensors were
#activated via the value they read off the paper
while(1):
    track = TR.AnalogRead() #create track array to house values read from track sensor, use AnalogRead to read and place values in track array
    print(track[0],track[1],track[2],track[3],track[4]) #print values of array for data collection
    
    #code run infinite loop, switiching ON-OFF between while loops based on sensor reading (track array values)
    #MOVE FORWARD
    #IR 3 black
    while((track[2] < V)):
        track = TR.AnalogRead() #continue to read current values of sensors for next loop
        print(track[0],track[1],track[2],track[3],track[4],"forward") #print current values of sensors for record,
                                                                      #also for troubleshooting any incorrect messages
        car.forward() #Bot moves according to sensor(s) activated
    
    #MOVE RIGHT    
    #IR 4-5 black    
    while((track[3] or track[4]) < V):
        track = TR.AnalogRead()
        print(track[0],track[1],track[2],track[3],track[4],"right")
        car.right()

    #MOVE LEFT
    #IR 1-2 black    
    while((track[0] or track[1]) < V):
        track = TR.AnalogRead()
        print(track[0],track[1],track[2],track[3],track[4],"left")
        car.left()
