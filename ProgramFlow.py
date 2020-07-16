import RPi.GPIO as GPIO
from time import sleep

#DC MOTOR
conveyorA = 21 
conveyorB = 18
cappingA1 = 26
cappingA2 = 29
cappingB1 = 22
cappingB2 = 24

#Load Sensor
pin1 = 31
pin2 = 32

#IR SENSOR
sensor = 16

#SERVO MOTOR
servoPin = 23
servo1 = 33
servo2 = 35
servo3 = 37
servo4 = 36
servo5 = 38
servo6 = 40
servo7 = 19

#STEPPER MOTOR
out1 = 13
out2 = 11
out3 = 15
out4 = 12
GPIO.setmode(GPIO.BOARD)

GPIO.setup(servoPin, GPIO.OUT)
GPIO.setup(servo1,GPIO.OUT)
GPIO.setup(servo2,GPIO.OUT)
GPIO.setup(servo3,GPIO.OUT)
GPIO.setup(servo4,GPIO.OUT)
GPIO.setup(servo5,GPIO.OUT)
GPIO.setup(servo6,GPIO.OUT)
GPIO.setup(servo7,GPIO.OUT)
GPIO.setup(sensor,GPIO.IN)
GPIO.setup(out1,GPIO.OUT)
GPIO.setup(out2,GPIO.OUT)
GPIO.setup(out3,GPIO.OUT)
GPIO.setup(out4,GPIO.OUT)
GPIO.setup(conveyorA,GPIO.OUT)
GPIO.setup(conveyorB,GPIO.OUT)
GPIO.setup(cappingA1,GPIO.OUT)
GPIO.setup(cappingA2,GPIO.OUT)
GPIO.setup(cappingB1,GPIO.OUT)
GPIO.setup(cappingB2,GPIO.OUT)

GPIO.output(conveyorA,GPIO.LOW)
GPIO.output(conveyorB,GPIO.LOW)

p = GPIO.PWM(servoPin, 50)
p.start(0)
def SetAngle(angle):
    duty = (angle / 18) + 3
    GPIO.output(servoPin,True)
    p.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(servoPin,False)
    p.ChangeDutyCycle(0)
    
def StepperMotor(x):
    i = 0
    positive = 0
    negative = 0 
    GPIO.output(out1,GPIO.LOW)
    GPIO.output(out2,GPIO.LOW)
    GPIO.output(out3,GPIO.LOW)
    GPIO.output(out4,GPIO.LOW)
    if x>0 and x<=400:
        for y in range(x,0,-1):
            if negative==1:
                if i==7:
                    i=0
                else:
                    i=i+1
                y=y+2
                negative=0
            positive=1
            #print((x+1)-y)
            if i==0:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                sleep(0.03)
                #time.sleep(1)
            elif i==1:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                sleep(0.03)
                #time.sleep(1)
            elif i==2:  
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                sleep(0.03)
                #time.sleep(1)
            elif i==3:    
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.LOW)
                sleep(0.03)
                #time.sleep(1)
            elif i==4:  
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.LOW)
                sleep(0.03)
                #time.sleep(1)
            elif i==5:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.HIGH)
                sleep(0.03)
                #time.sleep(1)
            elif i==6:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.HIGH)
                sleep(0.03)
                #time.sleep(1)
            elif i==7:    
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.HIGH)
                sleep(0.03)
                #time.sleep(1)
            if i==7:
                i=0
                continue
            i=i+1

    elif x<0 and x>=-400:
        x=x*-1
        for y in range(x,0,-1):
            if positive==1:
                if i==0:
                    i=7
                else:
                    i=i-1
                y=y+3
                positive=0
            negative=1
            print((x+1)-y) 
            if i==0:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                sleep(0.03)
                #time.sleep(1)
            elif i==1:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                sleep(0.03)
                #time.sleep(1)
            elif i==2:  
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                sleep(0.03)
                #time.sleep(1)
            elif i==3:    
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.LOW)
                sleep(0.03)
                #time.sleep(1)
            elif i==4:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.LOW)
                sleep(0.03)
                #time.sleep(1)
            elif i==5:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.HIGH)
                sleep(0.03)
                #time.sleep(1)
            elif i==6:    
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.HIGH)
                sleep(0.03)
                #time.sleep(1)
            elif i==7:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.HIGH)
                sleep(0.03)
                #time.sleep(1)
            if i==0:
                i=7
                continue
            i=i-1
        
keyboard_input = input()
keyboard_int = int(keyboard_input,10)

if keyboard_int==1: 
    # PROGRAM START
    #HOLDS STEPPER INTO PLACE
    GPIO.output(out1,GPIO.LOW)
    GPIO.output(out2,GPIO.LOW)
    GPIO.output(out3,GPIO.HIGH)
    GPIO.output(out4,GPIO.LOW)
    
    # START CONVEYORA
    GPIO.output(conveyorA,GPIO.HIGH)
    print( "Conveyor A running")
    
    # IR DETECTED
    while True:
        if GPIO.input(sensor):
            print ("No Object Detected!")
            while GPIO.input(sensor):
                sleep(0.2)
        else:
            print ("Object Detected")
            GPIO.output(conveyorA,GPIO.LOW)
            print ("Conveyor A stopped")
            break  
        sleep(0.5)
    
    # START STEPPER MOTOR STOPS at LOAD SENSOR
    StepperMotor(-60)
    print("Starts Stepper Motor to Load Sensor")
    
    # STARTS SERVO MOTOR
    SetAngle(90)
    print("Opens Servo Motor")
    sleep(5)
    # LOAD SENSOR DETECTS THE CORRECT WEIGHT
    SetAngle(0)  #CLOSES SERVO MOTOR
    print("Load Sensor Detected, Closes Servo Motor")
    sleep(5)
     
    # START STEPPER MOTOR STOPS at CAPPING
    StepperMotor(-110)
    print("Move Stepper Motor to Capping")
    
    #CAPPING STARTS
    print("Capping Working")
    GPIO.output(cappingA1,GPIO.LOW)
    GPIO.output(cappingA2,GPIO.HIGH)
    sleep(3)
    GPIO.output(cappingA1,GPIO.LOW)
    GPIO.output(cappingA2,GPIO.LOW)
    
    GPIO.output(cappingB1,GPIO.LOW)
    GPIO.output(cappingB2,GPIO.HIGH)
    sleep(3)
    GPIO.output(cappingB1,GPIO.LOW)
    GPIO.output(cappingB2,GPIO.LOW)
    
    GPIO.output(cappingA1,GPIO.HIGH)
    GPIO.output(cappingA2,GPIO.LOW)
    sleep(3)
    GPIO.output(cappingA1,GPIO.LOW)
    GPIO.output(cappingA2,GPIO.LOW)
    print("Capping Finished")
    
    #START STEPPER MOTOR STOPS at END of ROTARY
    StepperMotor(-180)
    print("Moves Stepper Motor to End of Rotary")
    
    #START CONVEYORB
    GPIO.output(conveyorB,GPIO.HIGH)
    sleep(8)
    GPIO.output(conveyorB,GPIO.LOW)
    print("Conveyor B Working")
    
    input = 0
    GPIO.cleanup()
else:
    GPIO.cleanup()
