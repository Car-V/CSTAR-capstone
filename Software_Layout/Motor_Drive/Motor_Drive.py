# Motor_Drive.py - Manges coordinated motor movement.
# This file contains the Motor_Drive class, which is used to manage the Motor class.


from Motor import Motor
from time import sleep

class Motor_Drive:
    def __init__(self):
        self.motorL = Motor(25, 11, 13, 6, 7) # motor 2, in1 in2 enb ch1 ch2
        self.motorR = Motor(10, 9, 12, 5, 8) # motor 1, in1 in2 enb ch1 ch2
        self.bot_idle()

    def bot_idle(self):
        print("we are decelerating")
        self.bot_decelerate()
        self.motorL.idle()
        self.motorR.idle()
        print("we are idle")
    
    def bot_forward(self):
        # really need to keep track that the duty cycles for the motors are the same/return to being the same when they should
        self.motorL.forward()
        self.motorR.forward()
        print("we are going forward")
        print("we are accelerating")
        self.bot_accelerate()
        
        for _ in range(10):  # Update PID in a loop
            self.motorL.update_speed()
            self.motorR.update_speed()
            sleep(0.1)  # Adjust timing as needed


    # go idle before going backward
    def bot_backward(self):
        self.motorL.backward()
        self.motorR.backward()
        print("we are going backward")
        print("we are accelerating")
        self.bot_accelerate()
        
        for _ in range(10):
            self.motorL.update_speed()
            self.motorR.update_speed()
            sleep(0.1)
        

    # go idle before making a turn
    def bot_clockwise(self):
        self.motorL.forward()
        self.motorR.backward()
        print("we are turning clockwise")
        print("we are accelerating")
        self.bot_accelerate()
        
        for _ in range(10):
            self.motorL.update_speed()
            self.motorR.update_speed()
            sleep(0.1)

    # go idle before making a turn
    def bot_counter_clockwise(self):
        self.motorL.backward()
        self.motorR.forward()
        print("we are turning counter-clockwise")
        print("we are accelerating")
        self.bot_accelerate()
        
        for _ in range(10):
            self.motorL.update_speed()
            self.motorR.update_speed()
            sleep(0.1)
        

    def bot_accelerate(self):
        while (Motor.duty_cycle < Motor.MAX_DUTY_CYCLE):
            print("Motor duty cycle is " + str(Motor.duty_cycle))
            Motor.accelerate()
            self.motorL.set_speed()
            self.motorR.set_speed()
            sleep(0.25)

    def bot_decelerate(self):
        while (Motor.duty_cycle > 0):
            print("Motor duty cycle is " + str(Motor.duty_cycle))
            Motor.decelerate()
            self.motorL.set_speed()
            self.motorR.set_speed()
            sleep(0.25)

    def delete(self):
        self.motorL.delete()
        print("deleted!")
    
    