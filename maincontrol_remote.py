import cv2
import numpy as np
from time import sleep  
import RPi.GPIO as GPIO
import pigpio
import Adafruit_DHT as dht
from gpiozero import DistanceSensor, Motor, Button
import evdev

from multiprocessing import Process

GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)

button_pin = 21
A1A_PIN = 23
A1B_PIN = 24
DHT_PIN = 19
servopin = 12 
remotepin = 27

GPIO.setup(A1A_PIN, GPIO.OUT)
GPIO.setup(A1B_PIN, GPIO.OUT)
GPIO.setup(servopin, GPIO.OUT)  # setting for servo pin
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

pi=pigpio.pi()

##############################################servo func#########################################
def ServoPos(degree):
    
    if degree > 180 :
        degree = 180
    elif degree < 0 : 
        degree = 0
    
    duty_cycle = int(500 + (degree / 180) * 2000)
    pi.set_servo_pulsewidth(servopin, duty_cycle)

############################################remote control#####################333#####################
def get_ir_device():
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if (device.name == "gpio_ir_recv"):
            return device
########################################movemotor###############################################33##
def move_motor(speed):
    motor = Motor(forward=A1A_PIN, backward=A1B_PIN)
    
    if speed > 0:
        motor.forward(speed)
    elif speed < 0:
        motor.backward(abs(speed))
    else:
        motor.stop()

#######################################tact switch func############################################3
def tactswitch():
   # GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
    return int(GPIO.input(button_pin)==GPIO.HIGH)
##############################################yolo func#############################################
def yolo(frame, size, score_threshold, nms_threshold):
    #  yolo network
    net = cv2.dnn.readNet(f"/home/pi/tri.ch/yolov3-tiny.weights","/home/pi/tri.ch/yolov3-tiny.cfg")
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

#######################################DC control############################################3
def DCcontrol():
    
    max_speed = 100
    min_speed = 30
    speed_step = 3
    
    current_speed = 0
    relative_temperature = 0
    auto = 0
    old_IRbutton = 0

    try:
        while 1:
            dev = get_ir_device()
            IRbutton = dev.read_one()
            pressed = tactswitch()
            sleep(0.1)
            if pressed==1 :
                if IRbutton!=0 and IRbutton == 67 and old_IRbutton == 0 :
                    auto = not auto
                    old_IRbutton = 67
                elif IRbutton == 0 :
                    old_IRbutton = 0
                    if auto == 1 :
                        humidity, temperature = dht.read_retry(dht.DHT11, DHT_PIN)
                        humidity = float(humidity)
                        temperature = float(temperature)
                        sensor = DistanceSensor(echo=17, trigger=22, max_distance=7)
                        distance1 = sensor.distance*1
                        print("checked distance = %.1fcm"%distance1)
                        sleep(0.1)                   

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
                            current_speed = (distance1-40)/7+70
                        elif relative_temperature > 25:
                            print('25')
                            current_speed = (distance1-40)/7+50
                        elif relative_temperature > 23:
                            print('23')
                            current_speed = (distance1-40)/7+30
                        else:
                            print('Low temperature')
                            current_speed = 40
                        move_motor(current_speed)     
                    elif auto == 0 :
                        if IRbutton != 0:
                            if IRbutton == 21: #up
                                current_speed += speed_step
                                if current_speed > max_speed:
                                    current_speed = max_speed
                            elif IRbutton == 7:  #down
                                current_speed -= speed_step
                                if current_speed < min_speed:
                                    current_speed = min_speed
                            move_motor(current_speed)
                        else :
                            move_motor(current_speed)
                        sleep(0.1)     
            else:
                auto = 0
                move_motor(0)
                break
    finally:
        GPIO.cleanup()
        

###############################servo control##################################################     

def Servocontrol():        
    ###########################################camera setting##################################
    # size list
    size_list = [320, 416, 608]
    #Loading web cam
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    ########################################algorithm setting#####################################
    coordinate_x_center = 500 #center of x coordinate
    coordinate_dis_av = 100 #allowed center displacement range
    angleofcameraview = 120 #max angle
    fullxsize = coordinate_x_center*2 #full size of x
   
    #####################################servo motor setting####################################
    pi.set_PWM_frequency(servopin, 50)
    startposition = 90 #start position of servo
    ServoPos(startposition) #move servo to the start position
    
    currentposition = startposition #set current position to the start positino
    auto=0
    rotate=0
    old_IRbutton=0
    
    while True :
        dev = get_ir_device()
        IRbutton = dev.read_one()
        pressed = tactswitch()
        if pressed==1:
            if IRbutton != 0 and IRbutton == 67 and old_IRbutton ==0 :
                auto = not auto
                old_IRbutton = 67
            elif IRbutton != 0 and IRbutton == 9 and old_IRbutton ==0 :
                rotate = not rotate
                old_IRbutton = 9 
            
            elif IRbutton == 0:
                old_IRbutton = 0
                if auto ==1 :
                    _,frame = cap.read()
                    coordinate = yolo(frame=frame, size=size_list[2], score_threshold=0.4, nms_threshold=0.4)
                    coordinate_x = coordinate[0]
                    
                    coordinate_x_distance = abs(coordinate_x-coordinate_x_center) #distance from center
                    finalposition = currentposition + (coordinate_x-coordinate_x_center/fullxsize*angleofcameraview) #final position func
                    
                    if cv2.waitKey(1) == ord('q'): ##quit
                        ServoPos(startposition) #move servo to the start position (reset)
                        break
                    elif (coordinate_x_distance > coordinate_dis_av) and coordinate_x !=0  :
                        ServoPos(finalposition)
                        currentposition = finalposition #current position update
                        sleep(1)
                    else : 
                        ServoPos(currentposition)

                elif auto ==0 :
                    if rotate == 0 :
                        ServoPos(currentposition)
                    elif rotate == 1 :
                        semifinalposition = (currentposition + 2)
                        if semifinalposition >180:
                            finalposition = 180 - semifinalposition%180
                        else :
                            finalposition == semifinalposition
                        ServoPos(finalposition)
                        currentposition = finalposition
        else : 
            ServoPos(startposition)
            auto = 0
            rotate = 0
            break
        
    cap.release()
    cv2.destroyAllWindows()

##########################################main func######################################
Servocontrol()

# ####
# p0 = Process(target=Servocontrol)
# p1 = Process(target=DCcontrol)

# while 1 : 
#     if tactswitch(button_pin)==1:
#         p0.start()
#        # p1.start()
#         p0.join()
#         #p1.join()
# ####