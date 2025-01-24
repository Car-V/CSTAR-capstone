from Motor_Drive import Motor_Drive
from time import sleep
class Motor_Drive_Manager:
    def __init__(self):
        self.Motor_Drive = Motor_Drive()
        self.Motor_Drive.idle()


    def test(self):
        self.Motor_Drive.forward()
        sleep(5)
        self.Motor_Drive.idle()
        sleep(2)
        self.Motor_Drive.backward()
        sleep(5)
        self.Motor_Drive.idle()
        sleep(2)
        self.Motor_Drive.clockwise()
        sleep(2)
        self.Motor_Drive.idle()
        sleep(2)
        self.Motor_Drive.counter_clockwise()
        sleep(2)
        self.Motor_Drive.idle()

def main():
    MDM = Motor_Drive_Manager()
    MDM.test()
    

if __name__ == "__main__":
    main()


