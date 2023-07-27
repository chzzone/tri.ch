from gpiozero import DistanceSensor, Motor, Button
import RPi.GPIO as GPIO
#######################################tact switch func############################################3
def tactswitch(button_pin):
   # GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 

   return int(GPIO.inpout(button_pin)==GPIO.HIGH)