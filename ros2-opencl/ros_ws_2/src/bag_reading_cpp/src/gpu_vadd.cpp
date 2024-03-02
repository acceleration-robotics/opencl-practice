#define CL_HPP_TARGET_OPENCL_VERSION 300

#include <iostream>
#include <fstream>
#include <string>
#include <CL/cl.hpp>

int main() {
    // Get available platform
    int v_size, errcode=0;
    std::cout << "Enter number of elements in vector : " << "\n";
    std::cin >> v_size;
    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    if (platforms.size() == 0)
    {
        std::cout << "No OpenCL platforms found" << std::endl;
        exit(1);
    }
    
    auto platform = platforms.front();
    std::cout << "Platform name : " << platform.getInfo<CL_PLATFORM_NAME>() << std::endl;
    // Get devices supported by platform and select one
    std::vector<cl::Device> devices;
    errcode = platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);
    if (devices.size() == 0)
    {
        std::cout << "No OpenCL devices found " << errcode << std::endl;
        exit(1);
    }
    auto device = devices.back();
    std::cout << "\t\tDevice Name: " << device.getInfo<CL_DEVICE_NAME>() << std::endl;  
    // Place selected device in a context
    int errorcode = 1;
    int * error = &errorcode;
    cl_uint ref_count;
    cl::Context context(device, NULL, NULL, NULL, error=error);
    if (*error == 0) {
        std::cout << "Context created successfully!" << "\n";
    }

    

    return 0;
}