## OpenCL Practice Repository

* Using OpenCL CPP, the following examples are implemented

-[x] Vector addition  
-[ ] Regression  
-[ ] Convolution

### Installing OpenCL on Ubuntu 22.04

* Verify that you have a CUDA-capable GPU
```
lspci | grep -i nvidia
```
* Remove existing nvidia driver
```
dpkg -l | grep nvidia-driver
sudo apt purge nvidia-driver-xxx
sudo apt autoremove
sudo apt autoclean
```
* Install the latest nvidia drivers for your system
```
sudo apt update
sudo apt install nvidia-driver-xxx
sudo reboot
```
* Check OpenCL version
```
/usr/bin/clinfo -l
```
* Install the cuda toolkit
```
sudo apt update && sudo apt upgrade
sudo apt install nvidia-cuda-toolkit
```
* Check cuda install
```
nvcc --version
```

### Running OpenCL CPP code
- On Ubuntu 22.04
* On terminal 1
```commandline
lttng-sessiond daemonize
lttng create vadd_session
lttng enable-event --userspace vadd:*
lttng start
```
* On terminal 2
```commandline
cd ~
git clone https://github.com/acceleration-robotics/opencl-practice.git
cd opencl-practice/vector_add/cpu_vadd_tracing
make all
./cpu_vadd
cd ../gpu_vadd_tracing
make all
./gpu_vadd
```
* On terminal 1 again
```commandline
lttng stop
babeltrace ~/lttng-traces/vadd_session*
```