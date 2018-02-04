#!/usr/bin/env python
import RPi.GPIO as GPIO
import datetime
import time


def on():
	GPIO.output(11, GPIO.HIGH)

def off():
	GPIO.output(11, GPIO.LOW)

def beep(x):
	on()
	time.sleep(x)
	off()

def destroy():
	GPIO.cleanup()                  

if __name__ == '__main__': 
	try:
            locked = False
            print("Script runs")
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(11, GPIO.OUT)
            
            while True:
                if GPIO.input(13) == False and locked == False:
                    print("Step: "+str(datetime.datetime.now()))
                    beep(0.2)
                    locked = True
                    
                if GPIO.input(13) == True and locked == True:
                    locked = False
                    time.sleep(3)

                    
	except KeyboardInterrupt:
	    print("Script stopped")
	    destroy()

