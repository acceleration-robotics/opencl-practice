#define CL_HPP_TARGET_OPENCL_VERSION 300

#include <iostream>
#include <fstream>
#include <string>
#include <CL/opencl.hpp> 
#include "./include/gpu_vadd_tp.h"

int main() {
    // Get available platform
    int v_size;
    std::cout << "Enter number of elements in vector : " << "\n";
    std::cin >> v_size;
    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    auto platform = platforms.front();
    // Get devices supported by platform and select one
    std::vector<cl::Device> devices;
    platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);
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

    // initialize vectors to be added on host memory (suffix - '_h')
    int A_h[v_size], B_h[v_size], C_h[v_size];
    for (int i=0; i<v_size; i++) {
        A_h[i] = rand() % 100;
        B_h[i] = rand() % 100;
    }
    // int A_h[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    // int B_h[] = { 10, 9, 8, 7, 6, 5, 4, 3, 2, 1 };
    // declare buffers on device memory (suffix - '_d'), C_d (R/W) = A_d (R) + B_d (R)
    cl::Buffer A_d(context, CL_MEM_READ_WRITE, sizeof(int) * v_size);
    cl::Buffer B_d(context, CL_MEM_READ_ONLY, sizeof(int) * v_size);
    cl::Buffer C_d(context, CL_MEM_WRITE_ONLY, sizeof(int) * v_size);

    // if (*error == 0) {
    //     std::cout << "Device Buffer declared successfully!" << "\n";
    // }

    // create command queue
    cl::CommandQueue queue(context, device);

    // initialize A_d and B_d using A_h and B_h
    // lttng_ust_tracepoint(vadd, gpu_tp, 0, "buffer write A_d");
    queue.enqueueWriteBuffer(A_d, CL_TRUE, 0, sizeof(int) * v_size, A_h);
    // lttng_ust_tracepoint(vadd, gpu_tp, 1, "end buffer write A_d");
    queue.enqueueWriteBuffer(B_d, CL_TRUE, 0, sizeof(int) * v_size, B_h);

    // if (*error == 0) {
    //     std::cout << "Device Buffer initialized successfully!" << "\n";
    // }
    
    // Read kernel code
    std::ifstream kernelfile("./kernels/gpu_vadd.cl");
    if (kernelfile.is_open()) {
        std::cout << "File open" << "\n";
    }
    // Save kernel code in string
    std::string kernelcode((std::istreambuf_iterator<char>(kernelfile)), std::istreambuf_iterator<char>());
    std::cout << kernelcode << "\n";

    // Initialize kernel source code
    cl::Program::Sources sources;
    sources.push_back({kernelcode.c_str(), kernelcode.length()+1});

    // Create OpenCL program to link source code to context
    cl::Program program(context, sources);

    // Compile OpenCL code in execution time
    if (program.build({device}) != CL_SUCCESS) {
        std::cout << "Error building program " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device) << std::endl;
        exit(1);
    } else {
        std::cout << "Program compiled succesfully!" << std::endl;
    }

    // Launch kernel on device
    // cl::Kernel kernel_add = cl::Kernel(program, "vector_add");
    // kernel_add.setArg(0, A_d);
    // kernel_add.setArg(1, B_d);
    // kernel_add.setArg(2, C_d);
    // queue.enqueueNDRangeKernel(kernel_add, 0, cl::NDRange(v_size), cl::NullRange);
    // queue.finish();

    auto vector_add = cl::KernelFunctor<cl::Buffer, cl::Buffer, cl::Buffer>(program, "vector_add");
    // cl::make_kernel<cl::Buffer, cl::Buffer, cl::Buffer> vector_add(program, "vector_add");
    cl::NDRange global(v_size);
    lttng_ust_tracepoint(vadd, gpu_tp, v_size, "gpu_vadd start");
    vector_add(cl::EnqueueArgs(queue, global), A_d, B_d, C_d);
    lttng_ust_tracepoint(vadd, gpu_tp, v_size, "gpu_vadd end");
    queue.finish();
    // lttng_ust_tracepoint(vadd, gpu_tp, 0, "End gpu vector add");
    queue.flush();

    // Read program output
    // int C_h[v_size];
    queue.enqueueReadBuffer(A_d, CL_TRUE, 0, sizeof(int)*v_size, C_h);
    for (int i=0; i<v_size; i++) {
        std::cout << C_h[i] << "\n";
    }

    return 0;
}