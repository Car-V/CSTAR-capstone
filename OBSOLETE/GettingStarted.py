import RPi.GPIO as GPIO          
from time import sleep

#Encoder Sets PWM
PWMhz = 1000
duty_cycle = 25

#Motor L
enL = 1 #Enable Inputs --> GPIO1
pin1L = 2 #GPIO2
pin2L = 3 #GPIO3

#Motor R
enR = 4 #Enable Inputs --> GPIO4
pin1R = 5 #GPIO5
pin2R = 6 #GPIO6

GPIO.setmode(GPIO.BCM) #Sets GPI mode for RPi --> maps pin numbers to GPIO numbers

GPIO.setup(pin1L,GPIO.OUT) #Sets GPIO24 as output
GPIO.setup(pin2L,GPIO.OUT) #Sets GPIO23 as output
GPIO.setup(enL,GPIO.OUT) #Sets GPIO25 as output

GPIO.setup(pin1R,GPIO.OUT) #Sets GPIO24 as output
GPIO.setup(pin2R,GPIO.OUT) #Sets GPIO23 as output
GPIO.setup(enR,GPIO.OUT) #Sets GPIO25 as output

pL=GPIO.PWM(enL,PWMhz) #Sets PWM for GPIO25 with 1000Hz frequency
pR=GPIO.PWM(enR,PWMhz) #Sets PWM for GPIO25 with 1000Hz frequency


pL.start(duty_cycle)
pR.start(duty_cycle)


#p=GPIO.PWM(en,1000) #Sets PWM for GPIO25 with 1000Hz frequency
#p.start(25) #Starts PWM with 25% duty cycle 
#######
#GPIO.setmode(GPIO.BCM) #Sets GPI mode for RPi --> maps pin numbers to GPIO numbers
#GPIO.setup(pin1,GPIO.OUT) #Sets GPIO24 as output
#GPIO.setup(pin2,GPIO.OUT) #Sets GPIO23 as output
#GPIO.setup(en,GPIO.OUT) #Sets GPIO25 as output
#GPIO.output(pin1,GPIO.LOW) #Sets GPIO24 to LOW --> motors are not running
#GPIO.output(pin2,GPIO.LOW) #Sets GPIO23 to LOW --> motors are not running

#consider SDA and SCL pins --> serial data and serial clock pins


#p=GPIO.PWM(en,1000) #Sets PWM for GPIO25 with 1000Hz frequency
#p.start(25) #Starts PWM with 25% duty cycle 
#right now motors cant turn, can just go forward or backward
print("\n")
print("The default speed & direction of motor is LOW & Forward.....") 
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

while(1):

    x=input() #accepts user input --> this is gonna be replaced with lidar data of some sort for autonomous driving
    if x=='r': #if user input is r, motor will run
        print("run")
        if(temp1==1): #if temp1 is 1, motor will run forward
         GPIO.output(pin1,GPIO.HIGH) #sets GPIO24 to HIGH --> motor runs forward
         GPIO.output(pin2,GPIO.LOW) #sets GPIO23 to LOW --> motor runs forward
         print("forward")
         x='z'
        else:
         GPIO.output(pin1,GPIO.LOW)
         GPIO.output(pin2,GPIO.HIGH)
         print("backward")
         x='z'


    elif x=='s':
        print("stop")
        GPIO.output(pin1,GPIO.LOW)
        GPIO.output(pin2,GPIO.LOW)
        x='z'

    elif x=='f':
        print("forward")
        GPIO.output(pin1,GPIO.HIGH)
        GPIO.output(pin2,GPIO.LOW)
        temp1=1
        x='z'

    elif x=='b':
        print("backward")
        GPIO.output(pin1,GPIO.LOW)
        GPIO.output(pin2,GPIO.HIGH)
        temp1=0
        x='z'

    elif x=='l':
        print("low")
        p.ChangeDutyCycle(25)
        x='z'

    elif x=='m':
        print("medium")
        p.ChangeDutyCycle(50)
        x='z'

    elif x=='h':
        print("high")
        p.ChangeDutyCycle(75) 
        x='z'
     
    
    elif x=='e':
        GPIO.cleanup()
        break
    
    else:
        print("<<<  wrong data  >>>")
        print("please enter the defined data to continue.....")