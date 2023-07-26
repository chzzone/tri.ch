import cv2

import numpy as np
import RPi.GPIO as GPIO 
from time import sleep  
import pigpio
import time

import Adafruit_DHT as dht
from gpiozero import DistanceSensor, Motor, InfraredSensor, Button

from multiprocessing import Process

GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM) 

button_pin = 21
A1A_PIN = 23
A1B_PIN = 24
DHT_PIN = 19
servopin = 12 
remotepin = 27

pi = pigpio.pi()
ir_receiver = InfraredSensor(remotepin)

##############################################yolo func#############################################
def yolo(frame, size, score_threshold, nms_threshold):
    #  yolo network
    net = cv2.dnn.readNet(f"/Users/han/vscode/python/yolov3-tiny.weights","/Users/han/vscode/python/yolov3-tiny.cfg")
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

    height, width, channels = frame.shape

        # blob
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (size, size), (0, 0, 0), True, crop=False)

        # import to network
    net.setInput(blob)

        # output
    outs = net.forward(output_layers)

        # lists
    class_ids = []
    confidences = []
    coordinates = []

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5 and class_id==0 :
                
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)

                coordinates.append([center_x,center_y])
                confidences.append(float(confidence))
                class_ids.append(class_id)
            else : 
                center_x1 = 0
                center_y1 = 0

    # Non Maximum Suppression
    indexes = cv2.dnn.NMSBoxes(coordinates, confidences, score_threshold=score_threshold, nms_threshold=nms_threshold)
    
    for i in range(len(coordinates)):
        if i in indexes:
            center_x1, center_y1 = coordinates[i]
            # print inf
            print(f"conf: {confidences[i]} / x: {center_x1} / y: {center_y1}")
    
    coordinate = [center_x1, center_y1]
    return coordinate

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
    

#######################################tact switch func############################################3
def tactswitch(button_pin):
   # GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 

    tact_switch = Button(button_pin)
    return int(tact_switch.is_pressed)

########movemotor###########
def move_motor(speed):
    motor = Motor(forward=A1A_PIN, backward=A1B_PIN)
    # 모터 제어 함수
    if speed > 0:
        motor.forward(speed)
    elif speed < 0:
        motor.backward(abs(speed))
    else:
        motor.stop()

#######################################DC control############################################3
def DCcontrol():
    GPIO.setup(A1A_PIN, GPIO.OUT)
    GPIO.setup(A1B_PIN, GPIO.OUT)
    
    current_speed = 0
    max_speed = 100
    min_speed = 30
    speed_step=10
    
    pressed=0
   

    auto=0

    try:
        while 1:
            pressed==tactswitch(button_pin)
            if pressed==1 :
                if ir_receiver.is_pressed and ir_receiver.value == 0xFF0000:
                    auto = not auto
                    
                elif auto ==1 :
                    humidity, temperature = dht.read_retry(dht.DHT11, DHT_PIN)

                    if humidity <= 50:
                        relative_temperature = temperature
                    elif humidity <= 60:
                        relative_temperature = temperature + 1
                    elif humidity <= 70:
                        relative_temperature = temperature + 2
                    elif humidity <= 80:
                        relative_temperature = temperature + 3
                    elif humidity <= 90:
                        relative_temperature = temperature + 4
                    print(relative_temperature)

                    if relative_temperature > 28:
                        print('28')
                        PWM_FREQ = 70
                        A1A = GPIO.PWM(A1A_PIN, PWM_FREQ)
                        A1B = GPIO.PWM(A1B_PIN, PWM_FREQ)
                        A1A.start(0)
                        A1A.ChangeDutyCycle((distance1-40)/7+70)
                    elif relative_temperature > 25:
                        print('25')
                        PWM_FREQ = 60
                        A1A = GPIO.PWM(A1A_PIN, PWM_FREQ)
                        A1B = GPIO.PWM(A1B_PIN, PWM_FREQ)
                        A1A.start(0)
                        A1A.ChangeDutyCycle((distance1-40)/7+70)
                    elif relative_temperature > 23:
                        print('23')
                        PWM_FREQ = 50
                        A1A = GPIO.PWM(A1A_PIN, PWM_FREQ)
                        A1B = GPIO.PWM(A1B_PIN, PWM_FREQ)
                        A1A.start(0)
                        A1A.ChangeDutyCycle((distance1-40)/7+70)
                        
                elif auto == 0 :
                    if ir_receiver.is_pressed:
                        if ir_receiver.value == 0xFF6897: #up
                            current_speed += speed_step
                            if current_speed > max_speed:
                                current_speed = max_speed
                        elif ir_receiver.value == 0xFF9867:  #down
                            current_speed -= speed_step
                            if current_speed < min_speed:
                                current_speed = min_speed
                        move_motor(current_speed)
                    else :
                        move_motor(current_speed)
                    sleep(0.1)
                    
            else:
                A1A.stop()
                break
    finally:
        GPIO.cleanup()
        

###############################servo control##################################################     

def Servocontrol():        
    GPIO.setup(servopin, GPIO.OUT)  # setting for servo pin
    ###########################################camera setting##################################
    # size list
    size_list = [320, 416, 608]
    #Loading web cam
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    ########################################algorithm setting#####################################
    coordinate_x_center = 500 #center of x coordinate
    coordinate_dis_av = 100 #center displacement range
    angleofcameraview = 120
    fullxsize = coordinate_x_center*2 #full size of x
   
    #####################################servo motor setting####################################
    pi.set_PWM_frequency(servopin, 50)
    startposition = 90 #start position of servo
    ServoPos(startposition) #move servo to the start position
    currentposition = startposition #set current position to the start positino
    
    
    while True :
        pressed = tactswitch(button_pin)
        auto=0
        rotate=0
        
        if pressed==1:
            if ir_receiver.is_pressed and ir_receiver.value == 0xFF0000:
                auto = not auto
            elif ir_receiver.is_pressed and ir_receiver.value == 0xFF0001:
                rotate = not rotate
                
            elif auto ==1 :
                _,frame = cap.read()
                coordinate = yolo(frame=frame, size=size_list[2], score_threshold=0.4, nms_threshold=0.4)
                coordinate_x = coordinate[0]
                
                coordinate_x_distance = abs(coordinate_x-coordinate_x_center) #distance from center
                finalposition = currentposition + (coordinate_x-coordinate_x_center/fullxsize*angleofcameraview) #final position func
                
                if cv2.waitKey(1) == ord('q'): ##quit
                    ServoPos(startposition) #move servo to the start position (reset)
                    break
                elif coordinate_x_distance > coordinate_dis_av and coordinate_x !=0  :
                    ServoPos(finalposition)
                    currentposition = finalposition #current position update
                    sleep(1) 

            elif auto ==0 :
                if rotate == 0 :
                    ServoPos(currentposition)
                elif rotate == 1 :
                    semifinalposition = (currentposition + 2)
                    if semifinalposition >180:
                        finalposition == 180 - semifinalposition%180
                    else :
                        finalposition == semifinalposition
                    ServoPos(finalposition)
                    currentposition = finalposition
        else : 
            ServoPos(startposition)
            break
        
    cap.release()
    cv2.destroyAllWindows()

##########################################main func######################################

p0 = Process(target=Servocontrol)
p1 = Process(target=DCcontrol)

while 1 : 
    if tactswitch(button_pin)==1:
        p0.start()
        p1.start()
        p0.join()
        p1.join()