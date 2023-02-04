import RPi.GPIO as GPIO
import time
import random
import picamera
import picamera.array
import numpy as np

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
	
		
	def setMotor(self, left, right):
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
            
#BCM port numbers for AlphaBot        
SERVO = 27
SERVOtop= 22
CS = 5
Clock = 25
Address = 24
DataOut = 23
DR = 16
DL = 19

#initializing GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SERVO, GPIO.OUT) 
GPIO.setup(SERVOtop, GPIO.OUT) 
#initializing GPIO
GPIO.setmode(GPIO.BCM) #setting mode to BCM pin names 
GPIO.setwarnings(False) #setting warnings to false to ignore them
GPIO.setup(Clock,GPIO.OUT) #setting up Clock, Address, CS, and DataOut respectively
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)
#IR avoidance
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DR,GPIO.IN,GPIO.PUD_UP)
GPIO.setup(DL,GPIO.IN,GPIO.PUD_UP)

p = GPIO.PWM(SERVO,50)
p2 = GPIO.PWM(SERVOtop,50)

car = AlphaBot()
speedL = 28
speedR = 23
car.setPWMA(speedL)
car.setPWMB(speedR)

#Camera servos sweep
p2.start(7.5)    		        #start at 90 degree
starting = 7.5    		        #starting point
increasement = 0.55555556       #increasement by 10 degree
#randomLeftRight = random.randint(0,1)   #random 0/1 - L/R
randomLeftRight = 0

#continuous loop
while(1):
	DR_status = GPIO.input(DR) 
	DL_status = GPIO.input(DL)
	with picamera.PiCamera() as camera: #code provided by professor, used to initialize and call PiCam to detect and preint light intensity
		camera.resolution = (128, 80)
		with picamera.array.PiRGBArray(camera) as stream:
			camera.exposure_mode = 'auto'
			camera.awb_mode = 'auto'
			camera.capture(stream, format='rgb')
			pixAverage = int(np.average(stream.array[...,1]))
	print ("Light Meter pixAverage=%i" % pixAverage) #print light intensity
	if (pixAverage < 25): #if pixaverage > 25, we determien we are looking at the light
		car.stop()
		while (1): 
			
			with picamera.PiCamera() as camera: #PiCam detection code
				camera.resolution = (128, 80)
				with picamera.array.PiRGBArray(camera) as stream:
					camera.exposure_mode = 'auto'
					camera.awb_mode = 'auto'
					camera.capture(stream, format='rgb')
					pixAverage = int(np.average(stream.array[...,1]))	
			print ("Light Meter pixAverage=%i" % pixAverage)
			if (randomLeftRight == 0):  #scanning Left
					print ("start Left")					
					p2.ChangeDutyCycle(starting)
					starting = starting - increasement  #increase point each time 10 degree
					time.sleep(1)
					if (pixAverage >= 25): #pixAverage >= 20
						print ("Light Left")
						car.right()
						time.sleep(0.3)
						car.stop()
						p2.start(7.5)
						break
					else:
						print (starting)
                        #Camera servos sweep
						print ("scanning Left")
						if (starting <= 3.61111108):    #reach limit angle (30) in Left
							print ("start Right")
							starting = 7.5              #set the cam to 90 and scan Right
							while (1):
								
								with picamera.PiCamera() as camera: #PiCam detection code
									camera.resolution = (128, 80)
									with picamera.array.PiRGBArray(camera) as stream:
										camera.exposure_mode = 'auto'
										camera.awb_mode = 'auto'
										camera.capture(stream, format='rgb')
										pixAverage = int(np.average(stream.array[...,1]))
								print ("Light Meter pixAverage=%i" % pixAverage)
								p2.ChangeDutyCycle(starting)
								starting = starting + increasement  #increase point each time 10 degree
								time.sleep(1)
								if (pixAverage >= 25): #pixAverage >= 20
									print ("Light Right")
									car.left()
									time.sleep(0.7)
									car.stop()
									p2.start(7.5)
									break   #break inner loop
								else:
									if (starting >= 10.83333336):
										starting = 7.5
										p2.start(7.5)
										randomLeftRight = random.randint(0,1)
										if (randomLeftRight == 0):
											car.right()
											time.sleep(0.7)
											car.stop()
											break
										else:
											car.left()
											time.sleep(0.7)
											car.stop()
											break
										break
			if (pixAverage >= 25):  #condition to exit outer loop when scan Right >= 20
				break
	
	else: #else we are not currently looking at the light and must move
		if((DL_status == 1) and (DR_status == 1)):
			car.forward()
			print("towards Light")
		elif((DL_status == 1) and (DR_status == 0)):
			car.stop()
			car.backward()
			time.sleep(0.2)
			car.stop()
			car.left()
			time.sleep(0.3)
			car.stop()
			print("left")
		elif((DL_status == 0) and (DR_status == 1)):
			car.stop()
			car.backward()
			time.sleep(0.2)
			car.stop()
			car.right()
			time.sleep(0.3)
			car.stop()
			print("right")
		else:
			car.stop()
			car.backward()
			time.sleep(0.2)
			car.stop()
			randomLeftRight2 = random.randint(0,1)
			if (randomLeftRight2 == 0):
				car.left()
				time.sleep(0.2)
				car.stop()
			else:
				car.right()
				time.sleep(0.2)
				car.stop()
			print("backward")
		
