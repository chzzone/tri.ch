import cv2
import numpy as np
from time import sleep  
import RPi.GPIO as GPIO
import pigpio
import Adafruit_DHT as dht
from gpiozero import DistanceSensor, Motor, Button
import evdev

from multiprocessing import Process

########################GPIO pin#####################
button_pin = 15
A1A_PIN = 23
A1B_PIN = 24
DHT_PIN = 19
servopin = 12 
remotepin = 27
distance_trigger = 22
distance_echo = 17

####################remote control num###########################
down_num=7
up_num=21
rotate_num=9
auto_num=67


######################################GPIO setup################
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)

GPIO.setup(A1A_PIN, GPIO.OUT)
GPIO.setup(A1B_PIN, GPIO.OUT)
GPIO.setup(servopin, GPIO.OUT)  # setting for servo pin
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


pi=pigpio.pi()

###############################dc motorsetup###############################
PWM_FREQ = 50
A1A = GPIO.PWM(A1A_PIN, PWM_FREQ)
A1A.start(0)
GPIO.output(A1B_PIN,0)
max_speed = 100
min_speed = 10
speed_step = 3
    
##################################camera info###########################
coordinate_x_center = 350 #center of x coordinate
coordinate_dis_av = 10 #allowed center displacement range
angleofcameraview = 150 #max angle
fullxsize = coordinate_x_center*2 #full size of x
   
#####################################servo motor setting################
    
########################################################################

#####################servo func#########################################
def ServoPos(degree):
    
    if degree > 180 :
        degree = 180
    elif degree < 0 : 
        degree = 0
    
    duty_cycle = int(500 + (degree / 180) * 2000)
    pi.set_servo_pulsewidth(servopin, duty_cycle)

############################################remote control##############
def get_ir_device():
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if (device.name == "gpio_ir_recv"):
            return device
########################################movemotor#######################
def move_motor(speed):
    A1A.ChangeDutyCycle(speed)
    if speed>max_speed :
        GPIO.output(A1A_PIN,1) 
        speed = max_speed
    elif speed<min_speed and speed>0 :
        GPIO.output(A1A_PIN,1) 
        speed = min_speed
    elif speed == 0 :
        speed = 0
        GPIO.output(A1A_PIN,0) 
    else : 
        speed = speed    

#######################################tact switch func############################################3
def tactswitch():
   # GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    high = (GPIO.input(button_pin)==GPIO.HIGH)
    sleep (0.1)
    return high
##############################################yolo func#############################################
def yolo(frame, size, score_threshold, nms_threshold):
    #  yolo network
    net = cv2.dnn.readNet(f"/home/RUSI/tri.ch/yolov3-tiny.weights","/home/RUSI/tri.ch/yolov3-tiny.cfg")
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

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
    boxes = []

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5 and class_id==0 :
                
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                #  top-left coordinate
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
            else : 
                center_x1 = 0
                center_y1 = 0

    # Non Maximum Suppression
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, score_threshold=score_threshold, nms_threshold=nms_threshold)
    
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            center_x1 = int(x+w/2)
            center_y1 = int(y+h/2)
    
    coordinate_yolo = [center_x1, center_y1]
    return coordinate_yolo

#######################################DC control############################################3
def DCcontrol():
    
    ##############################initial value##########################
    current_speed = int(max_speed + min_speed*2)/3
    relative_temperature = 0
    auto = 0
    old_IRbutton = 0
    IRbutton = 0
    IRdetect = 0
    old_signal = 0
    pressed = 0
    
    dev = get_ir_device()


    try:
        while 1:
            #print(current_speed)
            #print("Auto", auto)
            #event = dev.read_one()
            if (event) :
                IRdetect = event.value
                if (IRdetect !=0) :
                    IRbutton = IRdetect
            else :
                IRbutton = 0
                old_IRbutton = 0
            signal = tactswitch()
            if signal == 1 and old_signal ==0:
                pressed = not pressed
                old_signal = signal
            else : 
                pressed = pressed
                old_signal = signal
            
            if pressed==1 :
                if IRbutton!=0 and IRbutton == auto_num and old_IRbutton == 0 :
                    auto = not auto
                    old_IRbutton = auto_num
                    print(1)
                
                elif IRbutton == 0 :
                    old_IRbutton = 0
                    
                if auto == 1 :
                    print(2)
                    #humidity, temperature = dht.read_retry(dht.DHT11, DHT_PIN)
                    #humidity = float(humidity)
                    #temperature = float(temperature)
                    #sensor = DistanceSensor(distacne_echo, distance_trigger, max_distance=7)
                    #distance1 = sensor.distance*1
                    humidity = 60
                    temperature = 28
                    distance1=20
                    print("checked distance = %.1fcm"%distance1)

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

                    current_speed = abs(min_speed+((distance1)+relative_temperature-30)/70*(max_speed-min_speed))
                       
                elif auto == 0 :
                    print(2)
                    if IRbutton != 0 and old_IRbutton == 0 :
                        print(1)
                        if IRbutton == up_num : 
                            if current_speed > max_speed :
                                current_speed = max_speed
                            else :
                                current_speed = current_speed + speed_step
                        elif IRbutton == down_num :  
                            if current_speed < min_speed :
                                current_speed = min_speed
                            else : 
                                current_speed = current_speed - speed_step
                        old_IRbutton = 1
                        move_motor(current_speed)
                        
                    else :
                        move_motor(current_speed)
                            
            else:
                auto = 0
                move_motor(0)
                
    finally:
        GPIO.cleanup()
        

###############################servo control##################################################     

def Servocontrol():        
    ###########################################camera setting##################################
    # size list
    size_list = [320, 416, 608]
    #Loading web cam
    cap = cv2.VideoCapture(0)
    cap.set(3, 320)
    cap.set(4, 240)
    
    pi.set_PWM_frequency(servopin, 50)
    startposition = 90 #start position of servo
    ServoPos(startposition) #move servo to the start position
    
    currentposition = startposition #set current position to the start positino
    auto=0
    rotate=0
    old_IRbutton=0
    IRbutton =0
    IRdetect = 0 
    dev = get_ir_device()
    old_signal = 0
    pressed =0
    rotatedirection = 0
    
    while True :
        event = dev.read_one()
        if (event):
            IRdetect = event.value
            if (IRdetect!=0) :
                IRbutton = IRdetect
        else : 
            IRbutton = 0
            old_IRbutton = 0
        signal = tactswitch()
        if signal == 1 and old_signal == 0:
            pressed = not pressed 
            old_signal = signal
        else :
            pressed = pressed
            old_signal = signal
            
        print ("tact", pressed)
        print ("IR", IRbutton)
        print (event)
        print ("auto", auto)
        print ("rotate", rotate)
        if pressed==1:
            if IRbutton != 0 and IRbutton == auto_num and old_IRbutton ==0 :
                auto = not auto
                old_IRbutton = auto_num
            elif IRbutton != 0 and IRbutton == rotate_num and old_IRbutton ==0 :
                rotate = not rotate
                old_IRbutton = rotate_num
            
            elif IRbutton == 0:
                old_IRbutton = 0
                if auto ==1 :
                    _,frame = cap.read()
                    coordinate = yolo(frame=frame, size=size_list[0], score_threshold=0.4, nms_threshold=0.4)
                    coordinate_x = coordinate[0]
                    print('CO',coordinate)
                    print('fa',frame.shape)
                    coordinate_x_distance = abs(coordinate_x-coordinate_x_center) #distance from center
                    finalposition = 180-((coordinate_x/fullxsize)*180) #final position func
        
                    if (coordinate_x_distance > coordinate_dis_av) and coordinate_x !=0  :
                        ServoPos(finalposition)
                        currentposition = finalposition #current position update
                    else : 
                        ServoPos(currentposition)

                elif auto ==0 :
                    if rotate == 0 :
                        ServoPos(currentposition)
                    elif rotate == 1 :
                        if currentposition > 179 or currentposition <1 :
                            rotatedirection = not rotatedirection
                        else :
                            rotatedirection = rotatedirection
                        if rotatedirection == 0 :
                            finalposition = (currentposition + 2)
                        else :
                            finalposition = currentposition - 2
                            
                        ServoPos(finalposition)
                        currentposition = finalposition
        else : 
            ServoPos(startposition)
            auto = 0
            rotate = 0
          
        
    cap.release()
    cv2.destroyAllWindows()

##########################################main func######################################
###########################################camera setting##################################
   
# size list
size_list = [320, 416, 608]
#Loading web cam
cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)

pi.set_PWM_frequency(servopin, 50)
startposition = 90 #start position of servo
ServoPos(startposition) #move servo to the start position

currentposition = startposition #set current position to the start positino
auto=0
rotate=0
old_IRbutton=0
IRbutton =0
IRdetect = 0 
dev = get_ir_device()
old_signal = 0
pressed =0
rotatedirection = 0

try :  
    while True :
        event = dev.read_one()
        if (event):
            IRdetect = event.value
            if (IRdetect!=0) :
                IRbutton = IRdetect
        else : 
            IRbutton = 0
            old_IRbutton = 0
        signal = tactswitch()
        if signal == 1 and old_signal == 0:
            pressed = not pressed 
            old_signal = signal
        else :
            pressed = pressed
            old_signal = signal
            
        print ("tact", pressed)
        print ("IR", IRbutton)
        print (event)
        print ("auto", auto)
        print ("rotate", rotate)
        if pressed==1:
            if IRbutton != 0 and IRbutton == auto_num and old_IRbutton ==0 :
                auto = not auto
                old_IRbutton = auto_num
            elif IRbutton != 0 and IRbutton == rotate_num and old_IRbutton ==0 :
                rotate = not rotate
                old_IRbutton = rotate_num
            
            elif IRbutton == 0 :
                old_IRbutton = 0
                   
            if IRbutton != 0 and old_IRbutton == 0 :
                print(1)
                if IRbutton == up_num : 
                    if current_speed > max_speed :
                        current_speed = max_speed
                    else :
                        current_speed = current_speed + speed_step
                elif IRbutton == down_num :  
                    if current_speed < min_speed :
                        current_speed = min_speed
                    else : 
                        current_speed = current_speed - speed_step
                old_IRbutton = 1
                move_motor(current_speed)
                        
            else :
                move_motor(current_speed)    
            
            if auto ==1 :
                _,frame = cap.read()
                coordinate = yolo(frame=frame, size=size_list[0], score_threshold=0.4, nms_threshold=0.4)
                coordinate_x = coordinate[0]
                print('CO',coordinate)
                print('fa',frame.shape)
                coordinate_x_distance = abs(coordinate_x-coordinate_x_center) #distance from center
                finalposition = 180-((coordinate_x/fullxsize)*180) #final position func
    
                if (coordinate_x_distance > coordinate_dis_av) and coordinate_x !=0  :
                    ServoPos(finalposition)
                    currentposition = finalposition #current position update
                else : 
                    ServoPos(currentposition)

            elif auto ==0 :
                if rotate == 0 :
                    ServoPos(currentposition)
                elif rotate == 1 :
                    if currentposition > 179 or currentposition <1 :
                        rotatedirection = not rotatedirection
                    else :
                        rotatedirection = rotatedirection
                    if rotatedirection == 0 :
                        finalposition = (currentposition + 2)
                    else :
                        finalposition = currentposition - 2
                        
                    ServoPos(finalposition)
                    currentposition = finalposition
        else : 
            ServoPos(startposition)
            move_motor(0)
            auto = 0
            rotate = 0
          
finally :
    GPIO.cleanup()      
    cap.release()
    cv2.destroyAllWindows()