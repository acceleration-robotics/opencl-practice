# ip address at wlan0 : 192.168.50.172
# ip address of rb5 : 127.0.1.1
import cv2
import imagezmq
from datetime import datetime

# port = 'tcp://127.0.1.1:5556'
imageHub = imagezmq.ImageHub()

# initialize the dictionary which will contain  information regarding
# when a device was last active, then store the last time the check
# was made was now
lastActive = {}
lastActiveCheck = datetime.now()

# stores the estimated number of Pis, active checking period, and
# calculates the duration seconds to wait before making a check to
# see if a device was active
ESTIMATED_NUM_RB5 = 1
ACTIVE_CHECK_PERIOD = 10
ACTIVE_CHECK_SECONDS = ESTIMATED_NUM_RB5 * ACTIVE_CHECK_PERIOD
# start looping over all the frames
while True:
	# receive RPi name and frame from the RPi and acknowledge
	# the receipt
	print('in loop')
	(rb5Name, frame) = imageHub.recv_image()
	print('recd image')
	imageHub.send_reply(b'OK')
	# if a device is not in the last active dictionary then it means
	# that its a newly connected device
	if rb5Name not in lastActive.keys():
		print("[INFO] receiving data from {}...".format(rb5Name))
	# record the last active time for the device from which we just
	# received a frame
	lastActive[rb5Name] = datetime.now()
	# key = cv2.waitKey(1) & 0xFF
	
    # if current time *minus* last time when the active device check
	# was made is greater than the threshold set then do a check
	# print((datetime.now() - lastActiveCheck).seconds, 'seconds')
	# if (datetime.now() - lastActiveCheck).seconds > 3:
	# 	# loop over all previously active devices
	# 	for (rb5Name, ts) in list(lastActive.items()):
	# 		# remove the RPi from the last active and frame
	# 		# dictionaries if the device hasn't been active recently
	# 		if (datetime.now() - ts).seconds > ACTIVE_CHECK_SECONDS:
	# 			print("[INFO] lost connection to {}".format(rb5Name))
	# 			lastActive.pop(rb5Name)
	# 	# set the last active check time as current time
	# 	lastActiveCheck = datetime.now()
	# if the `q` key was pressed, break from the loop
	cv2.imshow("Frame", frame) 
	if cv2.waitKey(25) & 0xFF == ord('q'):
		break
# do a bit of cleanup
cv2.destroyAllWindows()