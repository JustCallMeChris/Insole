#!/usr/bin/env python
import RPi.GPIO as GPIO
import logging as log
import datetime
import time

log.basicConfig(filename='info.log',level=log.INFO, format='%(asctime)s %(message)s')
buzzerPin = 11
stepPin = 13

def on():
	GPIO.output(buzzerPin, GPIO.HIGH)

def off():
	GPIO.output(buzzerPin, GPIO.LOW)

def beep(x):
	on()
	time.sleep(x)
	off()

def destroy():
	GPIO.cleanup()                  

if __name__ == '__main__': 
	try:
	    log.info("Script is running.")
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(stepPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(buzzerPin, GPIO.OUT)
            
            while True:
                if GPIO.input(stepPin) == False:
		    log.info("Step: "+str(datetime.datetime.now()))
                    beep(0.2)
                    time.sleep(3) # stops logging next 3 seconds
                    
	except KeyboardInterrupt:
	    log.info("Script stopped.")
	    destroy()
