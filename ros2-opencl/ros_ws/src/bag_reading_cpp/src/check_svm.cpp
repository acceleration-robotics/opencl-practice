#include <iostream>
#include <CL/cl2.hpp>
#include <opencv2/opencv.hpp>

int main() {

    std::vector<cl::Platform> platforms;
	cl::Platform::get(&platforms);
	if (platforms.size() == 0)
	{
		std::cout << "No OpenCL platforms found" << std::endl;
		exit(1);
	}
	std::vector<cl::Device> devices;
	platforms[0].getDevices(CL_DEVICE_TYPE_GPU, &devices);
	// device = devices[0];
    cl::Device device = devices[0];
	std::cout << "Using device: " << device.getInfo<CL_DEVICE_NAME>() << std::endl;
	std::cout << "Using platform: " << platforms[0].getInfo<CL_PLATFORM_NAME>() << std::endl;
    cl::Context context(device);

    cl_device_svm_capabilities svmCapabilities = device.getInfo<CL_DEVICE_SVM_CAPABILITIES>();

    if (svmCapabilities & CL_DEVICE_SVM_FINE_GRAIN_BUFFER) {
        std::cout << "SVM Fine Grain Buffer is supported.\n";
    } else {
        std::cout << "SVM Fine Grain Buffer is not supported.\n";
    }

    if (svmCapabilities & CL_DEVICE_SVM_COARSE_GRAIN_BUFFER) {
        std::cout << "SVM Coarse Grain Buffer is supported.\n";
    } else {
        std::cout << "SVM Coarse Grain Buffer is not supported.\n";
    }

    if (svmCapabilities & CL_DEVICE_SVM_FINE_GRAIN_SYSTEM) {
        std::cout << "SVM Fine Grain System is supported.\n";
    } else {
        std::cout << "SVM Fine Grain System is not supported.\n";
    }

    if (svmCapabilities & CL_DEVICE_SVM_ATOMICS) {
        std::cout << "SVM Atomics is supported.\n";
    } else {
        std::cout << "SVM Atomics is not supported.\n";
    }

    return 0;
}