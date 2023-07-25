import cv2
import numpy as np

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
    font = cv2.FONT_HERSHEY_PLAIN
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            center_x1 = int(2*x+w)
            center_y1 = int(2*y+h)
            class_name = classes[class_ids[i]]
            label = f"{class_name} {confidences[i]:.2f}"
            color = colors[class_ids[i]]

            # drawing rectangles and writing texts
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.rectangle(frame, (x - 1, y), (x + len(class_name) * 13 + 65, y - 25), color, -1)
            cv2.putText(frame, label, (x, y - 8), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 2)
            
            # print inf
            print(f"[{class_name}({i})] conf: {confidences[i]} / x: {center_x1} / y: {center_y1}")
    
    coordinate = [center_x1, center_y1]
    return [coordinate, frame]
                    
                    
                    
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

# size list
size_list = [320, 416, 608]

#Loading web cam
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

while True :
    _,frame = cap.read()
    [coordinate, frame] = yolo(frame=frame, size=size_list[0], score_threshold=0.4, nms_threshold=0.4)
    
    cv2.imshow('Webcam', frame)
    
    if cv2.waitKey(1) == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()