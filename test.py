import  RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM) 

MOTOR1B = 23
MOTOR1E = 24
ena = 25

MOTOR2B = 16
MOTOR2E = 26
enb = 12

GPIO.setup(MOTOR1B, GPIO.OUT)
GPIO.setup(MOTOR1E, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)

GPIO.setup(MOTOR2B, GPIO.OUT)
GPIO.setup(MOTOR2E, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT)

pwmA = GPIO.PWM(ena, 100)
pwmB = GPIO.PWM(enb, 100)
pwmA.start(60)
pwmB.start(60)

GPIO.output(MOTOR1B,GPIO.HIGH)
GPIO.output(MOTOR1E, GPIO.LOW)

GPIO.output(MOTOR2B, GPIO.HIGH)
GPIO.output(MOTOR2E, GPIO.LOW)
time.sleep(10)
