# ip address at wlan0 : 192.168.50.172
# ip address of rb5 : 127.0.1.1
import cv2
import imagezmq
import argparse
import socket
import time
from imutils.object_detection import non_max_suppression 
import numpy as np

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-s", "--server-ip", required=True,
	help="ip address of the server to which the client will connect")
args = vars(ap.parse_args())
# initialize the ImageSender object with the socket address of the
# server
sender = imagezmq.ImageSender(connect_to="tcp://{}:5555".format(
	args["server_ip"]))

rb5Name = socket.gethostname()
print(rb5Name)
# vs = VideoStream(usePiCamera=True).start()
filename = './images/pedestrians_on_street_p.mp4'
# vs = VideoStream(src=0).start()
# vs = VideoStream(src=filename).start()
# cap = cv2.VideoCapture(filename)
cap = cv2.VideoCapture(2)
print("created videowriter")
time.sleep(2.0)

while cap.isOpened():
         
    # Capture one frame at a time
    success, frame = cap.read() 
    print('read frame')
    # sender.send_image(rb5Name, frame)
	# read the frame from the camera and send it to the server
    # frame = vs.read()
    if success :
        print('frame success')

        # Detect people
        # image: a single frame from the video
        # winStride: step size in x and y direction of the sliding window
        # padding: no. of pixels in x and y direction for padding of 
        # sliding window
        # scale: Detection window size increase coefficient   
        # bounding_boxes: Location of detected people
        # weights: Weight scores of detected people
        # Tweak these parameters for better results
        (bounding_boxes, weights) = hog.detectMultiScale(frame, 
                                                        winStride=(4, 4),
                                                        padding=(4, 4), 
                                                        scale=1.1)
    
        # Draw bounding boxes on the frame
        for (x, y, w, h) in bounding_boxes: 
                cv2.rectangle(frame, 
                (x, y),  
                (x + w, y + h),  
                (0, 0, 255), 
                2)
                            
        # Get rid of overlapping bounding boxes
        # You can tweak the overlapThresh value for better results
        bounding_boxes = np.array([[x, y, x + w, y + h] for (
                                    x, y, w, h) in bounding_boxes])
        print('bb : ', len(bounding_boxes))
                
        selection = non_max_suppression(bounding_boxes, 
                                        probs=None, 
                                        overlapThresh=0.45)
        print('s : ', len(selection))
            
        # draw the final bounding boxes
        for (x1, y1, x2, y2) in selection:
            cv2.rectangle(frame, 
                        (x1, y1), 
                        (x2, y2), 
                        (0, 255, 0), 
                        4)

        sender.send_image(rb5Name, frame)
        print("sent frame")
    else : 
        break

cap.release()
cv2.destroyAllWindows() 