"""
DEMS.py in Roller Tank Project.

DEMS.py is the basic controller of two motors, using PWM.

"""

import RPi.GPIO as GPIO

class DEMS:
    def __init__(self, speed=0, steer=0):
        self.speed = speed
        self.steer = steer

        # BCM GPIO number
        self.leftgo = 20
        self.leftback = 21
        self.leftpwm = 16

        self.rightgo = 19
        self.rightback = 26
        self.rightpwm = 13

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.leftgo,GPIO.OUT)
        GPIO.setup(self.leftback, GPIO.OUT)
        GPIO.setup(self.leftpwm, GPIO.OUT)
        GPIO.setup(self.rightgo, GPIO.OUT)
        GPIO.setup(self.rightback, GPIO.OUT)
        GPIO.setup(self.rightpwm, GPIO.OUT)
        self.leftp = GPIO.PWM(self.leftpwm, 1000)
        self.rightp = GPIO.PWM(self.rightpwm, 1000)

    def start(self):
        self.leftp.start(self.speed)
        self.rightp.start(self.speed)

    def stop(self):
        self.leftp.stop()
        self.rightp.stop()

    def set_speed(self, speed):
        self.speed = speed
        self.update_pwm()

    def set_steer(self, steer):
        self.steer = steer
        self.update_pwm()

    def update_pwm(self):
        if(self.speed == 0):
            self.leftp.ChangeDutyCycle(0)
            self.rightp.ChangeDutyCycle(0)
        if(self.speed > 0):
            GPIO.output(self.leftgo, GPIO.LOW)
            GPIO.output(self.leftback, GPIO.HIGH)
            GPIO.output(self.rightgo, GPIO.LOW)
            GPIO.output(self.rightback, GPIO.HIGH)
            if(self.steer > 0):
                self.rightp.ChangeDutyCycle(self.speed*(1-self.steer))
                self.leftp.ChangeDutyCycle(self.speed)
            if(self.steer == 0):
                self.rightp.ChangeDutyCycle(self.speed)
                self.leftp.ChangeDutyCycle(self.speed)
            if(self.steer < 0):
                self.rightp.ChangeDutyCycle(self.speed)
                self.leftp.ChangeDutyCycle(self.speed*(1-self.steer))

        if(self.speed < 0):
            GPIO.output(self.leftgo, GPIO.HIGH)
            GPIO.output(self.leftback, GPIO.LOW)
            GPIO.output(self.rightgo, GPIO.HIGH)
            GPIO.output(self.rightback, GPIO.LOW)
            if(self.steer > 0):
                self.rightp.ChangeDutyCycle(-self.speed)
                self.leftp.ChangeDutyCycle(-self.speed*(1+self.steer))
            if(self.steer == 0):
                self.rightp.ChangeDutyCycle(-self.speed)
                self.leftp.ChangeDutyCycle(-self.speed)
            if(self.steer < 0):
                self.rightp.ChangeDutyCycle(-self.speed*(1+self.steer))
                self.leftp.ChangeDutyCycle(-self.speed)


