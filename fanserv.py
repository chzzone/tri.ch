import cv2
import numpy as np
import RPi.GPIO as GPIO 
from time import sleep  

#################################################yolo func#################################################
def yolo(frame, size, score_threshold, nms_threshold):
    #  yolo network
    net = cv2.dnn.readNet(f"/Users/han/vscode/python/yolov3-tiny.weights","/Users/han/vscode/python/yolov3-tiny.cfg")
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

    # random RGB
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

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
            class_name = classes[class_ids[i]]
            # print inf
            print(f"[{class_name}({i})] conf: {confidences[i]} / x: {center_x1} / y: {center_y1}")
    
    coordinate = [center_x1, center_y1]
    return coordinate

##############################################servo func#########################################
def setServoPos(degree):
    if degree > 180 :
        degree = 180
    elif degree < 0 : 
        degree = 0
    duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    print("Degree: {} to {}(Duty)".format(degree, duty))

    servo.ChangeDutyCycle(duty)

                           
########################################classes#################################################                  
classes = ["person", "bicycle", "car", "motorcycle",
           "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant",
           "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
           "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
           "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis",
           "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
           "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife",
           "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog",
           "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
           "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
           "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
           "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]


###########################################camera setting##########################################
# size list
size_list = [320, 416, 608]
#Loading web cam
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

########################################motor setting###############################################
servoPin          = 12   # servo motor pin num
SERVO_MAX_DUTY    = 12   # cycle length of max pos
SERVO_MIN_DUTY    = 3    # cycle length of min pos

GPIO.setmode(GPIO.BOARD)        # GPIO setting
GPIO.setup(servoPin, GPIO.OUT)  # setting for servo pin

servo = GPIO.PWM(servoPin, 50)  # PWM 50HZ 20ms
servo.start(0)  # start servo PWM, if duty=0, servo not working



########################################main func###################################################
coordinate_x_center = 500 #center of x coordinate
coordinate_dis_av = 100 #center displacement range
startposition = 90 #start position of servo
angleofcameraview = 120
fullxsize = coordinate_x_center*2 #full size of x

setServoPos(startposition) #move servo to the start position
currentposition = startposition #set current position to the start positino

while True :
    _,frame = cap.read()
    coordinate = yolo(frame=frame, size=size_list[2], score_threshold=0.4, nms_threshold=0.4)
    coordinate_x = coordinate[0]
    
    coordinate_x_distance = abs(coordinate_x-coordinate_x_center) #distance from center
    finalposition = currentposition + (coordinate_x-coordinate_x_center/fullxsize*angleofcameraview) #final position func
    
    if cv2.waitKey(1) == ord('q'): ##quit
        setServoPos(startposition) #move servo to the start position (reset)
        break
    elif coordinate_x_distance > coordinate_dis_av and coordinate_x !=0  :
        setServoPos(finalposition)
        currentposition = finalposition #current position update
        sleep(1) 
    
cap.release()
cv2.destroyAllWindows()