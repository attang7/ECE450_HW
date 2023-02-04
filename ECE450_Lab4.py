#----------
#Authors: Anthony Tang, Bao Phan
#ECE 450 Assignment Lab 4
#Due Date: October 21, 2022
#----------
#Title: PID Line Follower
#Description: Test robot mobility via PID control for line following
#----------

import RPi.GPIO as GPIO
import time
	
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
		self.numSensors = numSensors
		self.calibratedMin = [0] * self.numSensors
		self.calibratedMax = [1023] * self.numSensors
		self.last_value = 0
		
	"""
	Reads the sensor values into an array. There *MUST* be space
	for as many values as there were sensors specified in the constructor.
	Example usage:
	unsigned int sensor_values[8];
	sensors.read(sensor_values);
	The values returned are a measure of the reflectance in abstract units,
	with higher values corresponding to lower reflectance (e.g. a black
	surface or a void).
	"""
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
		
	"""
	Reads the sensors 10 times and uses the results for
	calibration.  The sensor values are not returned; instead, the
	maximum and minimum values found over time are stored internally
	and used for the readCalibrated() method.
	"""
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

	"""
	Returns values calibrated to a value between 0 and 1000, where
	0 corresponds to the minimum value read by calibrate() and 1000
	corresponds to the maximum value.  Calibration values are
	stored separately for each sensor, so that differences in the
	sensors are accounted for automatically.
	"""
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
		
		#print("readCalibrated",sensor_values)
		return sensor_values
			
	"""
	Operates the same as read calibrated, but also returns an
	estimated position of the robot with respect to a line. The
	estimate is made using a weighted average of the sensor indices
	multiplied by 1000, so that a return value of 0 indicates that
	the line is directly below sensor 0, a return value of 1000
	indicates that the line is directly below sensor 1, 2000
	indicates that it's below sensor 2000, etc.  Intermediate
	values indicate that the line is between two sensors.  The
	formula is:

	   0*value0 + 1000*value1 + 2000*value2 + ...
	   --------------------------------------------
			 value0  +  value1  +  value2 + ...

	By default, this function assumes a dark line (high values)
	surrounded by white (low values).  If your line is light on
	black, set the optional second argument white_line to true.  In
	this case, each sensor value will be replaced by (1000-value)
	before the averaging.
	"""
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
car = AlphaBot()
IR = TRSensor()

#V = the value we determien that seperates white from black for track sensors (white > V, Black < V)
V = 950

#our values for KP, KI, KD for PID control
KP = 0.09
KI = 0.00067
KD = 0.004

#other values used in computing P, I, and D
LastError = 0
ErrorSum = 0
ErrorDiff = 0
newSpeedA = 0
newSpeedB = 0
Goal = 2000	


speed = 25	# Let max speedis 24 PWM
car.forward()

#while loop to continuously go around track
while(1):
	track = IR.AnalogRead()
    
    #MOVE FORWARD
    #IR 3 black
	while((track[2] < V)):
		track = IR.AnalogRead() #AnalogRead to keep reading sensor values to determine while loop to enter
		position = IR.readLine() #readLine to determine distance from line
		error = Goal - position #error = Goal value (2000) - current postion, where error = 0 is on the line
		ErrorSum = ErrorSum+error #determine error values for I(sum of instantaneous errors) and D(derivative of errors) equations
		ErrorDiff = error - LastError
		lastError = error
		P = KP*error #determine P, I, D from above values
		I = KI*ErrorSum
		D = KD*ErrorDiff
		newSpeedA = speed-(P+I+D) #new speed of wheels based on PID
		newSpeedB = speed+(P+I+D)
		print ("A: " + str(newSpeedA)) #print speeds for record
		print ("B- " + str(newSpeedB))
		if (newSpeedA <= 0):
			newSpeedA = 0
		if (newSpeedB <= 0):
			newSpeedB = 0
		if (newSpeedA > speed):
			newSpeedA = speed
		if (newSpeedB > speed):
			newSpeedB = speed
			
		car.setPWMA(newSpeedA) #set new PWM for bot to move
		car.setPWMB(newSpeedB)

		
    #MOVE RIGHT    
    #IR 4-5 black    
	while((track[3] or track[4]) < V):
		track = IR.AnalogRead()
		position = IR.readLine()
		error = Goal - position
		ErrorSum = ErrorSum+error
		ErrorDiff = error - LastError
		lastError = error
		P = KP*error
		I = KI*ErrorSum
		D = KD*ErrorDiff
		newSpeedB = speed+(P+I+D)
		if (newSpeedB <= 0):
			newSpeedB = 0
		if (newSpeedB > speed):
			newSpeedB = speed
		car.setPWMA(0)
		car.setPWMB(newSpeedB)

    #MOVE LEFT
    #IR 1-2 black    
	while((track[0] or track[1]) < V):
		track = IR.AnalogRead()
		position = IR.readLine()
		error = Goal - position
		ErrorSum = ErrorSum+error
		ErrorDiff = error - LastError
		lastError = error
		P = KP*error
		I = KI*ErrorSum
		D = KD*ErrorDiff
		newSpeedA = speed-(P+I+D)
		if (newSpeedA <= 0):
			newSpeedA = 0
		if (newSpeedA > speed):
			newSpeedA = speed
		car.setPWMA(newSpeedA)
		car.setPWMB(0)
