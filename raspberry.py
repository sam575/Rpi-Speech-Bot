#Code works on speech commands
#For manual control make changes in Xbee as mentioned in Xbee function 

#importing basic libraries to control the bot hardware
import RPi.GPIO as GPIO
import time
import socket
import sys
from xbee import XBee
from serial import Serial

#change your port according to your laptop. It may be same
#This is the port in your laptop to which the XBee is connected.
PORT = '/dev/ttyAMA0'
BAUD = 9600
ser = Serial(PORT, BAUD)
xbee = XBee(ser)

#****************************************#
#importing the necessary packages
#for image processing tasks
from picamera.array import PiRGBArray     #As there is a resolution problem in raspberry pi, will not be able to capture frames by VideoCapture
from picamera import PiCamera
import cv2
import numpy as np

#Functions for image processing
def segment_colour(frame):    #returns only the red colors in the frame
    #for segmenting the ball or object
    hsv_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_1 = cv2.inRange(hsv_roi, np.array([160, 160,10]), np.array([190,255,255]))
    ycr_roi=cv2.cvtColor(frame,cv2.COLOR_BGR2YCrCb)
    mask_2=cv2.inRange(ycr_roi, np.array((0.,165.,0.)), np.array((255.,255.,255.)))

    mask = mask_1 | mask_2
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode  = np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,kern_erode)      #Eroding
    mask=cv2.dilate(mask,kern_dilate)     #Dilating
    #cv2.imshow('mask',mask)
    return mask

def find_blob(blob): #returns the red colored circle
    #returns the largest contour encircling the detected object
    largest_contour=0
    cont_index=0
    _,contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area >largest_contour) :
            largest_contour=area
           
            cont_index=idx
            #if res>15 and res<18:
            #    cont_index=idx
                              
    r=(0,0,2,2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])
       
    return r,largest_contour

def target_hist(frame):
    hsv_img=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   
    hist=cv2.calcHist([hsv_img],[0],None,[50],[0,255])
    return hist

#CAMERA CAPTURE
#initialize the camera and grab a reference to the raw camera capture
# camera = PiCamera()
# camera.resolution = (160, 120)
# camera.framerate = 16
# rawCapture = PiRGBArray(camera, size=(160, 120))
 
# # allow the camera to warmup
# time.sleep(0.001)
#*********************************************#

#****************DEFINING********************
#Assigining the pins for the both servos of tilt pan(Change according to your Rpi)   
#common ground taken out 
#Grounds of battery and and rpi to be made common
servo_s = 3
servo_u = 5

enable1=31
enable2=33

#Ultrasonic Sensor pins 
#Powered with 5V pins of Rpi
#Common ground taken out
trig1 = 35
trig2 = 37                               #1 is for ultrasonic sensor in the front, 2 is for ultrasonic sensor in the back
echo1 = 36
echo2 = 38
threshold = 0

#Defining the pins for the left and right motor
#common ground taken out
#Ground of motor driver and rpi to be made common
lmotor1 = 16          
lmotor2 = 18
rmotor1 = 22
rmotor2 = 15

#*************CONFIGURING***********************
#Setting the board to address them via pins
GPIO.setmode(GPIO.BOARD)

#Confuring them to respective functions
GPIO.setup(servo_s,GPIO.OUT)
GPIO.setup(servo_u,GPIO.OUT)

GPIO.setup(trig1,GPIO.OUT)
GPIO.setup(trig2,GPIO.OUT)
GPIO.setup(echo1,GPIO.IN)
GPIO.setup(echo2,GPIO.IN)

GPIO.setup(lmotor1,GPIO.OUT)
GPIO.setup(lmotor2,GPIO.OUT)
GPIO.setup(rmotor1,GPIO.OUT)
GPIO.setup(rmotor2,GPIO.OUT)

#enable motor pins
GPIO.setup(enable1,GPIO.OUT)
GPIO.setup(enable2,GPIO.OUT)

#*******************INTIALIZING********************
#Initializing the servo motion & direction
pwm_s_servo = GPIO.PWM(servo_s,100)
pwm_s_servo.start(15)
pwm_u_servo = GPIO.PWM(servo_u,100)
pwm_u_servo.start(9)
time.sleep(.3)
pwm_s_servo.stop(0)
pwm_u_servo.stop(0)
prev_duty_s  = 15
prev_duty_u  = 9

#Initializing the Ultrasonice
GPIO.output(trig1,0)
GPIO.output(trig2,0)	
GPIO.output(enable1,1)
GPIO.output(enable2,1)

#Initializing the motors
pwm_l_motor_1 = GPIO.PWM(lmotor1,30)
pwm_l_motor_1.start(0)
pwm_l_motor_2 = GPIO.PWM(lmotor2,30)
pwm_l_motor_2.start(0)
pwm_r_motor_1 = GPIO.PWM(rmotor1,30)
pwm_r_motor_1.start(0)
pwm_r_motor_2 = GPIO.PWM(rmotor2,30)
pwm_r_motor_2.start(0)

#**************SERVO FUNCTIONS*************************
def  servo_set_angle(servo,angle):
 		pwm_servo = GPIO.PWM(servo,100)
		time.sleep(.1)
		duty = 5 + angle/9
                pwm_servo.start(duty)
                time.sleep(.5)
		pwm_s_servo.stop(0)
		GPIO.output(servo,0)

def servo_rotate(servo,pres_angle,rot_angle):
		pwm_servo = GPIO.PWM(servo,100)
		time.sleep(.1)
		angle = pres_angle + rot_angle
		duty = 5 + angle/9
                pwm_servo.start(duty)
                time.sleep(.5)
		pwm_s_servo.stop(0)
		GPIO.output(servo,0)

#***************ULTRASONIC SENSOR FUNCTIONS************************

def ultrasonic_check(ECHO,TRIG):
	    #Sending the signal
        GPIO.output(TRIG,1)
        time.sleep(0.001)
        GPIO.output(TRIG,0)
        
        #Taking the feedback
        while GPIO.input(ECHO) == 0:
               pass
 	start= time.time()

        while GPIO.input(ECHO) == 1:
               pass
        stop = time.time()
        d= (stop -start) * 170
        print d 
        return (stop-start)*170

def ULTRA_DATA():
	    while 1: 
	    	u1= ultrasonic_check(echo1,trig1)   #checking front wheel conditions
                time.sleep(0.1)
                u2= ultrasonic_check(echo2,trig2)
                time.sleep(0.1)
                if (u1<threshold and u2 > threshold):
                        data= "f"
                        PROGRAM(data)  
                elif (u1>threshold and u2 < threshold):
                        data= "b"
                        PROGRAM(data)
                elif (u1 < threshold and u2 < threshold):
                        data= "stop"
                        PROGRAM(data)

#***************MOTOR FUNCTIONS************************

def forward(x):
        GPIO.output(enable1,1)
        GPIO.output(enable2,1)
	pwm_l_motor_1.ChangeDutyCycle(0)
	pwm_l_motor_2.ChangeDutyCycle(x)
	pwm_r_motor_1.ChangeDutyCycle(x)
	pwm_r_motor_2.ChangeDutyCycle(0)

def backward(x):
        GPIO.output(enable1,1)
        GPIO.output(enable2,1)
	pwm_l_motor_1.ChangeDutyCycle(x)
	pwm_l_motor_2.ChangeDutyCycle(0)
	pwm_r_motor_1.ChangeDutyCycle(0)
	pwm_r_motor_2.ChangeDutyCycle(x)

def left():
        GPIO.output(enable1,1)
        GPIO.output(enable2,1)
	pwm_l_motor_1.ChangeDutyCycle(50)
	pwm_l_motor_2.ChangeDutyCycle(0)
     	pwm_r_motor_1.ChangeDutyCycle(50)
	pwm_r_motor_2.ChangeDutyCycle(0)

def right():
        GPIO.output(enable1,1)
        GPIO.output(enable2,1)
	pwm_l_motor_1.ChangeDutyCycle(0)
	pwm_l_motor_2.ChangeDutyCycle(50)
	pwm_r_motor_1.ChangeDutyCycle(0)
	pwm_r_motor_2.ChangeDutyCycle(50)

def stop():
        GPIO.output(enable1,1)
        GPIO.output(enable2,1)
	pwm_l_motor_1.ChangeDutyCycle(0)
	pwm_l_motor_2.ChangeDutyCycle(0)
	pwm_r_motor_1.ChangeDutyCycle(0)
	pwm_r_motor_2.ChangeDutyCycle(0)

# The bot is initially at rest
data = "stop"
     
#*****************xbee function***************************
def XBEE_DATA():
            #For keyboard control use raw_input instead of xbee read.
            #Type the shortcut commands for keyboard control(Eg:f,fas,lor) 
        		#data= raw_input("input")
			      data = xbee.wait_read_frame()
        		print data 
	    			#ser.close()
        		return data
#*******&&**&&**&&--FINAL_CODE--&&**&&**&&*****************
def  PROGRAM():
            
            global prev_data            
            print "In data loop"
            #Set initial angle for servos
  	        prev_s_angle = 90
            prev_u_angle = 30 #TO BE CALIBRATED
            x=50
            
            data= XBEE_DATA()
            print data
            print "prev data"
        
            time.sleep(0.1)                       
                      
            if data == "f":     #command to move forward                                      
                        forward(50)

            elif data == "b" :  
            	        backward(50)
                                        
            
            elif data == "tl":  # turn left
                        left()
                        time.sleep(2)
                        stop()                                   
                                                                     
            elif data == "tr":  #turn right
                        right()
                        time.sleep(2)
                        stop()
                                   
            elif data == "lir":  #little right
            	        right()
                        time.sleep(1)
                        stop()

            elif data == "lil":  #little left
            	        left()
                        time.sleep(1)
                        stop()

            elif data == "lim":  #little more
                        if (prev_data == "tl"  or prev_data == "lil"): 
                        	left()
                        	time.sleep(.5)
                        	stop()

                        if (prev_data ==  "tr" or prev_data ==  "lir"): 
                        	right()
                        	time.sleep(.5)
                        	stop()

            elif data == "fas":      #faster
		        if prev_data == "f":
                        	forward(90)

                        if prev_data == "b" :
                        	backward(90)
                        	
            elif data == "nor":     #normal speed
		        if prev_data == "f" : 
                        	forward(50)

                        if prev_data ==  "b":
                           	backward(50)
                        	
            elif data == "tb":      #turn back
                        right()
                        time.sleep(4)
                        stop()

            elif data == "stop"  :
            	        stop()
                        
  #*********************CAMERA MOTION COMMANDS*******************#
            elif data == "lor": #look right
                     servo_set_angle(servo_s ,30)
                     prev_s_angle=150

            elif data == "lol": #look left
                     servo_set_angle(servo_s ,150)
                     prev_s_angle=30

            elif data == "lolir":  #look little right
                     servo_rotate(servo_s ,prev_s_angle,-30)
                     prev_s_angle= prev_s_angle - 30

            
            elif data == "lolil": #look little left
                     servo_rotate(servo_s ,prev_s_angle,+30)
                     prev_s_angle= prev_s_angle + 30

      	    elif data == "lolim": #look little more

                     if (prev_data == "lol" or  prev_data== "lolil") :
                      		servo_rotate(servo_s ,prev_s_angle,+10)
                     		prev_s_angle= prev_s_angle  + 10

                     elif (data == "lor"  or data == "lolir" ): 
                        	servo_rotate(servo_s ,prev_s_angle,-10)
                     		prev_s_angle= prev_s_angle - 10
            
            elif data == "lot":     # to look top/up
                     servo_set_angle(servo_u ,15)
                     prev_u_angle=15

                     
            elif data == "los":               #TO look STRAIGHT
            	     servo_set_angle(servo_s ,90)
                     prev_s_angle=90
                     servo_set_angle(servo_u ,30)
                     prev_u_angle=30
#**************************************************************************#
            #Find and track red/orange ball/object
            elif data == "find red" :
                     count = 0
                     camera = PiCamera()
                     camera.resolution = (160, 120)
                     camera.framerate = 16
                     rawCapture = PiRGBArray(camera, size=(160, 120))
 
                     # allow the camera to warmup
                     time.sleep(0.001)

                     flag = 0

                     for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                                #grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
                                frame = image.array
                                frame=cv2.flip(frame,1)
                                global centre_x
                                global centre_y
                                centre_x=0.
                                centre_y=0.
                                hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                                mask_red=segment_colour(frame)      #masking red the frame
                                loct,area=find_blob(mask_red)
                                x,y,w,h=loct
                                    
                                if (w*h) < 10:
                                      found=0
                                else:
                                      found=1
                                      simg2 = cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
                                      centre_x=x+((w)/2)
                                      centre_y=y+((h)/2)
                                      cv2.circle(frame,(int(centre_x),int(centre_y)),3,(0,110,255),-1)
                                      centre_x-=80
					#check if centre_y is correct
                                      centre_y=6-centre_y
                                      print 'centre_x:',centre_x,'centre_y:',centre_y,'area:',area
                                initial=400

                                print 'count:', count,'flag:', flag

                                if(found==0):
                                      #if the ball is not found and the last time it sees ball in which direction, it will start to rotate in that direction
                                      if flag==0:
                                            right()
                                            time.sleep(0.05)
                                      else:
                                            left()
                                            time.sleep(0.05)
                                      stop()
                                      time.sleep(0.0125)
                                      count = count + 1
                                      #If ball not found after few turns, bot moves forward and starts turning again.
                                      #Its a Random search. Needs optimization
                                      if count > 20 :
                                          forward(50)
                                          time.sleep(0.1)
                                          count= 0
                               
                                elif(found==1):
                                      count = 0
                                      if(area<initial):
                                                  if(centre_x>=30):
                                                        right()
                                                        time.sleep(0.00625)
                                                        stop()
                                                        time.sleep(0.0125)
                                                        forward(50)
                                                        time.sleep(0.00625)
                                                        stop()
                                                        time.sleep(0.0125)
                                                        left()
                                                        time.sleep(0.00625)
                                                        flag = 1
                                                  elif(centre_x<=-30):
                                                        left()
                                                        time.sleep(0.00625)
                                                        stop()
                                                        time.sleep(0.0125)
                                                        forward(50)
                                                        time.sleep(0.00625)
                                                        stop()
                                                        time.sleep(0.0125)
                                                        right()
                                                        time.sleep(0.00625)
                                                        stop()
                                                        time.sleep(0.0125)
                                                        flag = 0
                                                  else:
                                                        forward(50)
                                                        time.sleep(0.01)
                                      elif(area>=initial):
                                            initial2=5000
                                            if(area<initial2):
                                                        #it brings coordinates of ball to center of camera's imaginary axis.
                                                        if(centre_x<=-30 or centre_x>=30):
                                                              if(centre_x<0):
                                                                    flag=0
                                                                    right()
                                                                    time.sleep(0.025)
                                                              elif(centre_x>0):
                                                                    flag=1
                                                                    left()
                                                                    time.sleep(0.025)
                                                        forward(50)
                                                        time.sleep(0.00003125)
                                                        stop()
                                                        time.sleep(0.00625)
                                            else:
                                                  #if it finds the ball and it is too close it lights up the led.
                                                  #GPIO.output(LED_PIN,GPIO.HIGH)
                                                  time.sleep(0.1)
                                                  stop()
                                                  time.sleep(0.1)
                                cv2.imshow("draw",frame)
                                cv2.imshow("Frame", mask_red)
                                key = cv2.waitKey(1) & 0xFF
             
                                # clear the stream in preparation for the next frame
                                rawCapture.truncate(0)
                           
                              # if the `q` key was pressed, break from the loop
                                if key == ord("q") :
                                      break
                     cv2.destroyAllWindows()
                     #findred()
                     camera.close()
                     print "Loop break from find red"
                     #print centre_x

            #stores previous command
            prev_data = data
            print prev_data
                                         
while True:
      try:
           PROGRAM()

      except KeyboardInterrupt:
           GPIO.cleanup()

GPIO.cleanup()
