# Human Detection using Qualcomm RB5
- This package contains scripts that can be used to detect the presence of humans/pedestrians from a live video feed obtained from a camera connected to the Qualcomm RB5. 
- There are two scripts : 
1. [streamer_client.py](./streamer_client.py) : RB5 acts as an ImageMQ client, takes in live video feed from the camera connected to the RB5, performs human detection using HOG descriptor, adds bounding box around detected human and sends the modified frame over TCP to the server (which is the companion computer)
2. [streamer_server.py](./streamer_server.py) : The companion computer acts as a server, receives video feed from the connected RB5 over TCP and displays the live video feed on the companion computer.

## Prequisites
- OpenCV
- [ImageZMQ](https://github.com/jeffbass/imagezmq)

## Installation
- In adb shell, run the following commands
```
pip3 install opencv-contrib-python imagezmq imutils
```
- On your companion computer, run the following : 
```
pip3 install opencv-contrib-python imagezmq imutils
```

Note : To find your server hostname and ip, you can go to the python console in command line and use the socket library to find it. The commands are as follows :
```
python3
>> import socket
>> socket.gethostname()
{>> accel-legion}
socket.gethostbyname('accel-legion')
{>> # IP Address}
```
## Running
- In adb shell, run the following commands
```
cd /root/DEVELOPMENT/opencl-test-codes/human_detection
python3 streamer_client.py --server-ip accel-legion
```
- In companion computer shell, run the following commands 
```
cd ~/anushree/opencl-test-codes/opencl-practice/human_detection
python3 streamer_server.py
```

