#----------
#Authors: Anthony Tang, Bao Phan
#ECE 450 Assignment Lab 1
#Due Date: September 19, 2022
#----------
#Title: Robot Mobility Test Version B
#Description: Test robot mobility/motot functions
#----------

import RPi.GPIO as GPIO
import time

class AlphaBot(object):
	
	def __init__(self,in1=12,in2=13,ena=6,in3=20,in4=21,enb=26):
		self.IN1 = in1 #initialize variables
		self.IN2 = in2
		self.IN3 = in3
		self.IN4 = in4
		self.ENA = ena
		self.ENB = enb

		GPIO.setmode(GPIO.BCM)  #initialize GPIO ports, IN1,IN2,IN3,IN4, ENA, ENB
		GPIO.setwarnings(False)
		GPIO.setup(self.IN1,GPIO.OUT)  
		GPIO.setup(self.IN2,GPIO.OUT)
		GPIO.setup(self.IN3,GPIO.OUT)
		GPIO.setup(self.IN4,GPIO.OUT)
		GPIO.setup(self.ENA,GPIO.OUT)
		GPIO.setup(self.ENB,GPIO.OUT)
		self.forward()
		self.PWMA = GPIO.PWM(self.ENA,500)  #set pwm
		self.PWMB = GPIO.PWM(self.ENB,500)
		self.PWMA.start(50)
		self.PWMB.start(50)

	def forward(self):
		GPIO.output(self.IN1,GPIO.HIGH)     #from Alphabot manual 
		GPIO.output(self.IN2,GPIO.LOW)      #1 0 0 1 robots forward
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH)

	def stop(self):
		GPIO.output(self.IN1,GPIO.LOW)      #0 0 0 0 robot stop
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.LOW)

	def backward(self):
		GPIO.output(self.IN1,GPIO.LOW)      #0 1 1 0 robot backwards
		GPIO.output(self.IN2,GPIO.HIGH)
		GPIO.output(self.IN3,GPIO.HIGH)
		GPIO.output(self.IN4,GPIO.LOW)

	def left(self):
		GPIO.output(self.IN1,GPIO.LOW)      #0 1 0 1, opposite wheels turn robot left
		GPIO.output(self.IN2,GPIO.HIGH)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH)

	def right(self):
		GPIO.output(self.IN1,GPIO.HIGH)     #1 0 1 0, opposite wheels turn robot right
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.HIGH)
		GPIO.output(self.IN4,GPIO.LOW)
		
	def setPWMA(self,value):                #set pwm for ENA
		self.PWMA.ChangeDutyCycle(value)

	def setPWMB(self,value):                #set pwm for ENB
		self.PWMB.ChangeDutyCycle(value)	
		
	def setMotor(self, left, right):        #not used
		if((right >= 0) and (right <= 100)):
			GPIO.output(self.IN1,GPIO.HIGH)
			GPIO.output(self.IN2,GPIO.LOW)
			self.PWMA.ChangeDutyCycle(right)
		elif((right < 0) and (right >= -100)):
			GPIO.output(self.IN1,GPIO.LOW)
			GPIO.output(self.IN2,GPIO.HIGH)
			self.PWMA.ChangeDutyCycle(0 - right)
		if((left >= 0) and (left <= 100)):
			GPIO.output(self.IN3,GPIO.HIGH)
			GPIO.output(self.IN4,GPIO.LOW)
			self.PWMB.ChangeDutyCycle(left)
		elif((left < 0) and (left >= -100)):
			GPIO.output(self.IN3,GPIO.LOW)
			GPIO.output(self.IN4,GPIO.HIGH)
			self.PWMB.ChangeDutyCycle(0 - left)

	
print('start')
car = AlphaBot()
#--------------------------------------------
car.setPWMA(22) #35 25
car.setPWMB(23.5) #38 28
car.forward()
time.sleep(5)   #program run time 
car.stop()
time.sleep(1)
car.backward()
time.sleep(10)
car.stop()
time.sleep(1)
car.forward()
time.sleep(5)
car.stop()
time.sleep(1)
#right 90
#-----sprin to the right-------
car.setPWMA(28) #35 25
car.setPWMB(28) #38 28
car.right()
time.sleep(0.5)
car.stop()
time.sleep(1)
car.setPWMA(22) #35 25
car.setPWMB(23.5) #38 28
car.forward()
time.sleep(5)
car.stop()
time.sleep(1)
car.backward()
time.sleep(5)
car.stop()
time.sleep(1)
#left 180
#######################################
car.setPWMA(28) #35 25
car.setPWMB(28) #38 28
car.left()
time.sleep(0.83)
car.stop()
time.sleep(1)
car.setPWMA(22) #35 25
car.setPWMB(23.5) #38 28
car.forward()
time.sleep(5)
car.stop()
time.sleep(1)
car.backward()
time.sleep(5)
car.stop()
time.sleep(1)
#right 90
#-----sprin to the right-------
car.setPWMA(28) #35 25
car.setPWMB(28) #38 28
car.right()

print('end')
