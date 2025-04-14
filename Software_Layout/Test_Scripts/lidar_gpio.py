#import pigpio
# JUST USE USB SERIAL
import RPi.GPIO as GPIO   
from time import sleep

TX_PIN = 14
RX_PIN = 15
GPIO.setmode(GPIO.BCM)

def read_data():
	#count, data = pi.bb_serial_read(RX_PIN)
	count, data = GPIO.bb_serial_read(RX_PIN)
	return data

def main():
	#pi = pigpio.pi()

	# pi.set_mode(TX_PIN, pigpio.OUTPUT)
	# pi.set_mode(RX_PIN, pigpio.INPUT)
	GPIO.setmode(TX_PIN, GPIO.OUT)
	GPIO.setmode(RX_PIN, GPIO.IN)

	#pi.bb_serial_read_open(RX_PIN, 115200)
	GPIO.bb_serial_read_open(RX_PIN, 115200)

	try:
		while True:
			data = read_data()
			if data:
				print(data)
			sleep(0.1)
	except KeyboardInterrupt:
		pass
	finally:
		#pi.bb_serial_read_close(RX_PIN)
		#pi.stop()
		GPIO.bb_serial_read_close(RX_PIN)
		GPIO.cleanup()		

if __name__ == "__main__":
	main()