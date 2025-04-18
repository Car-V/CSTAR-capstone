# Motor_Drive_Manager.py - runs predefined movement tests.
# This file contains the Motor_Drive_Manager class, which is used to manage the Motor_Drive class.


from Motor_Drive import Motor_Drive
from time import sleep

class Motor_Drive_Manager:
    def __init__(self):
        self.Motor_Drive = Motor_Drive()
        self.Motor_Drive.bot_idle()

    def test(self):
        self.Motor_Drive.bot_forward()
        sleep(5)
        
        self.Motor_Drive.bot_idle()
        sleep(2)
        
        self.Motor_Drive.bot_backward()
        sleep(5)
        
        self.Motor_Drive.bot_idle()
        sleep(2)
        
        self.Motor_Drive.bot_clockwise()
        sleep(2)
        
        self.Motor_Drive.bot_idle()
        sleep(2)
        
        self.Motor_Drive.bot_counter_clockwise()
        sleep(2)
        
        self.Motor_Drive.bot_idle()
        self.Motor_Drive.delete()

def main():
    MDM = Motor_Drive_Manager()
    MDM.test()
    

if __name__ == "__main__":
    main()


