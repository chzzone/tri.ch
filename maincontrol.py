import cv2

import numpy as np
import RPi.GPIO as GPIO 
from time import sleep  

import Adafruit_DHT as dht
from gpiozero import DistanceSensor

import lirc

from multiprocessing import Process

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
    
    duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    print("Degree: {} to {}(Duty)".format(degree, duty))

    return duty
    

#######################################tact switch func############################################3
def tactswitch(button_pin):
    GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
    
    if GPIO.input(button_pin) == GPIO.HIGH:
        a=1  
    else :
        a=0
    return a

#######################################DC control############################################3
def DCcontrol(A1A_PIN,A1B_PIN,DHT_PIN,button_pin):
    GPIO.setup(A1A_PIN, GPIO.OUT)
    GPIO.setup(A1B_PIN, GPIO.OUT)
    pressed=0

    try:
        while 1:
            pressed==tactswitch(button_pin)
            if pressed==1 :
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
                    print('상대온도 28도씨 조과')
                    PWM_FREQ = 70
                    A1A = GPIO.PWM(A1A_PIN, PWM_FREQ)
                    A1B = GPIO.PWM(A1B_PIN, PWM_FREQ)
                    A1A.start(0)
                    A1A.ChangeDutyCycle((distance1-40)/7+70)
                elif relative_temperature > 25:
                    print('상대온도 25도씨 조과')
                    PWM_FREQ = 60
                    A1A = GPIO.PWM(A1A_PIN, PWM_FREQ)
                    A1B = GPIO.PWM(A1B_PIN, PWM_FREQ)
                    A1A.start(0)
                    A1A.ChangeDutyCycle((distance1-40)/7+70)
                elif relative_temperature > 23:
                    print('상대온도 23도씨 조과')
                    PWM_FREQ = 50
                    A1A = GPIO.PWM(A1A_PIN, PWM_FREQ)
                    A1B = GPIO.PWM(A1B_PIN, PWM_FREQ)
                    A1A.start(0)
                    A1A.ChangeDutyCycle((distance1-40)/7+70)
            else:
                A1A.stop()
                break

    finally:
        GPIO.cleanup()
        

###############################servo control##################################################     

def Servocontrol(servoPin,button_pin):        
    GPIO.setup(servoPin, GPIO.OUT)  # setting for servo pin
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
    servo = GPIO.PWM(servoPin, 50)  # PWM 50HZ 20ms
    startposition = 90 #start position of servo
    servo.start(0)  # start servo PWM, if duty=0, servo not working

    servo.ChangeDutyCycle(ServoPos(startposition)) #move servo to the start position
    currentposition = startposition #set current position to the start positino
    pressed=0
    
    ################################Lric setting################
    
    while True :
        if pressed==1:
            _,frame = cap.read()
            coordinate = yolo(frame=frame, size=size_list[2], score_threshold=0.4, nms_threshold=0.4)
            coordinate_x = coordinate[0]
            
            coordinate_x_distance = abs(coordinate_x-coordinate_x_center) #distance from center
            finalposition = currentposition + (coordinate_x-coordinate_x_center/fullxsize*angleofcameraview) #final position func
            
            if cv2.waitKey(1) == ord('q'): ##quit
                servo.ChangeDutyCycle(ServoPos(startposition)) #move servo to the start position (reset)
                break
            elif coordinate_x_distance > coordinate_dis_av and coordinate_x !=0  :
                servo.ChangeDutyCycle(ServoPos(finalposition))
                currentposition = finalposition #current position update
                sleep(1) 
        else : 
            servo.ChangeDutyCycle(ServoPos(startposition))
            break
        
    cap.release()
    cv2.destroyAllWindows()
    
    
##########################################main func######################################
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM) 

Button_PIN = 21
A1A_PIN = 23
A1B_PIN = 24
DHT_PIN = 19
servoPin = 12 

p0 = Process(target=Servocontrol,args=(servoPin,Button_PIN))
p1 = Process(target=DCcontrol,args=(A1A_PIN,A1B_PIN,DHT_PIN,Button_PIN))

p0.start()
p1.start()
p0.join()
p1.join()
