import pigpio
pi=pigpio.pi()
servopin = 12 
##############################################servo func#########################################
def ServoPos(degree):

    SERVO_MAX_DUTY    = 12   # cycle length of max pos
    SERVO_MIN_DUTY    = 3    # cycle length of min pos
    
    if degree > 180 :
        degree = 180
    elif degree < 0 : 
        degree = 0
    
    duty_cycle = int(500 + (degree / 180) * 2000)
    pi.set_servo_pulsewidth(servopin, duty_cycle)
