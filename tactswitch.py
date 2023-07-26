from gpiozero import DistanceSensor, Motor, Button
#######################################tact switch func############################################3
def tactswitch(button_pin):
   # GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
    tact_switch = Button(button_pin)
    return int(tact_switch.is_pressed)