#include <iostream>
#include <CL/cl2.hpp>
#include <opencv2/opencv.hpp>
#include <bag_reading_cpp/utils.hpp>

int main() {
    cl_int error = 0;
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
    
    cl::CommandQueue r_queue(context, device, 0, NULL); // Read command queue
    cl::CommandQueue w_queue(context, device, 0, NULL); // Write command queue
    cl::CommandQueue k_queue(context, device, 0, NULL); // Kernel enqueue command queue

    std::string add_kerneldir = get_current_dir() + "/kernels/gpu_vadd.cl";
    std::string add_kernelcode = readFile(add_kerneldir);

    cl::Program::Sources sources;
    sources.push_back({add_kernelcode.c_str(), add_kernelcode.length()+1});

    cl::Program program(context, sources);

    if (program.build({device}) != CL_SUCCESS) {
        std::cout << "Error building program " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device) << std::endl;
        exit(1);
    } else {
        std::cout << "Program compiled succesfully!" << std::endl;
    }

    cv::Mat image = cv::imread("/home/user/DEVELOPMENT/ros_ws/saved_images/resize_image_0.png");
    int v_size = image.total();
    // int v1[] = { 10, 9, 8, 7, 6, 5, 4, 3, 2, 1 };
    // int v2[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    // int v3[] = { 20, 29, 28, 27, 26, 25, 24, 23, 22, 21 };
    // int v4[] = { 10, 11, 12, 13, 14, 15, 16, 17, 18, 9 };

    cl::Kernel krnl_1(program, "copy", &error);
    assert(error == CL_SUCCESS);
    cl::Kernel krnl_2(program, "copy", &error);
    assert(error == CL_SUCCESS);

    // Create device side buffers
    cl::Buffer device_input_cl(context, CL_MEM_READ_ONLY, sizeof(cl_uchar)*v_size, NULL, &error);
    assert(error == CL_SUCCESS);
    cl::Buffer device_temp_cl(context, CL_MEM_READ_WRITE, sizeof(cl_uchar)*v_size, NULL, &error);
    assert(error == CL_SUCCESS);
    cl::Buffer device_output_cl(context, CL_MEM_WRITE_ONLY, sizeof(cl_uchar)*v_size, NULL, &error);
    assert(error == CL_SUCCESS);

    // Create host side buffers with memory allocated on the host
    // Data will be copied over to pinned host ptrs that the host side buffer is mapped to
    cl::Buffer host_input_cl(context, CL_MEM_ALLOC_HOST_PTR, sizeof(cl_uchar)*v_size, NULL, &error);
    assert(error == CL_SUCCESS);
    cl::Buffer host_output_cl(context, CL_MEM_ALLOC_HOST_PTR, sizeof(cl_uchar)*v_size, NULL, &error);
    assert(error == CL_SUCCESS);

    // Set kernel arguments
    error = krnl_1.setArg(0, device_input_cl);
    assert(error == CL_SUCCESS);
    error = krnl_1.setArg(1, device_temp_cl);
    assert(error == CL_SUCCESS);
    error = krnl_2.setArg(0, device_temp_cl);
    assert(error == CL_SUCCESS);
    krnl_2.setArg(1, device_output_cl);
    assert(error == CL_SUCCESS);

    // Synchronization events
    cl::Event input_event, output_event, krnl1_event, krnl2_event;
    // Map host input buffer to pinned memory on the host
    cl_uchar * pinned_host_input = (cl_uchar*)w_queue.enqueueMapBuffer(host_input_cl, CL_TRUE, CL_MAP_WRITE, 0, sizeof(cl_uchar)*v_size, NULL, NULL, &error);
    // Populate pinned host input
    std::memcpy(pinned_host_input, image.data, sizeof(cl_uchar)*v_size);
    assert(pinned_host_input != NULL);
    std::cout << int(pinned_host_input[3]) << std::endl;

    // Map host output buffer to pinned memory on host
    cl_uchar * pinned_host_output = (cl_uchar*)w_queue.enqueueMapBuffer(host_output_cl, CL_FALSE, CL_MAP_READ, 0, sizeof(cl_uchar)*v_size, NULL, NULL, &error);
    assert(pinned_host_output != NULL);
    
    // Send the first set of data in
    error = w_queue.enqueueWriteBuffer(device_input_cl, CL_TRUE, 0, sizeof(cl_uchar)*v_size, pinned_host_input, NULL, &input_event);
    input_event.wait();
    assert(error == CL_SUCCESS);
    
    // Enqueue kernels
    error = k_queue.enqueueNDRangeKernel(krnl_1, 0, cl::NDRange(v_size), cl::NullRange, NULL, &krnl1_event);
    krnl1_event.wait();
    assert(error == CL_SUCCESS);
    error = k_queue.enqueueNDRangeKernel(krnl_2, 0, cl::NDRange(v_size), cl::NullRange, NULL, &krnl2_event);
    krnl2_event.wait();
    assert(error == CL_SUCCESS);
    k_queue.finish();
    k_queue.flush();
    // Read buffer
    error = r_queue.enqueueReadBuffer(device_output_cl, CL_FALSE, 0, sizeof(cl_uchar)*v_size, pinned_host_output, NULL, &output_event);
    output_event.wait();
    assert(error == CL_SUCCESS);
    std::cout << "Read buffer : \n";
    std::cout << int(pinned_host_output[3]) << std::endl;
    // for (int i=0; i<v_size; i++) {
    //     std::cout << pinned_host_output[i] << "\n";
    // }
    r_queue.finish();
    r_queue.flush();
    
    w_queue.finish();
    w_queue.flush();

    return 0;
}