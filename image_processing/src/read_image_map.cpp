#define CL_HPP_TARGET_OPENCL_VERSION 200
#include "utils.hpp"
#include <iostream>
#include <cassert>

int main(int argc, char *argv[]) {


    auto start = std::chrono::high_resolution_clock::now();
    std::vector<cl::Platform> platforms;
	cl::Platform::get(&platforms);
	if (platforms.size() == 0)
	{
		std::cout << "No OpenCL platforms found" << std::endl;
		exit(1);
	}
	std::vector<cl::Device> devices;
	platforms[0].getDevices(CL_DEVICE_TYPE_GPU, &devices);
	cl::Device device = devices[0];
	std::cout << "Using device: " << device.getInfo<CL_DEVICE_NAME>() << std::endl;
	std::cout << "Using platform: " << platforms[0].getInfo<CL_PLATFORM_NAME>() << std::endl;
	cl::Context context(device);

    // Set input, output image dirs and kernel dir
    std::string input_dir = get_current_dir() + "/input_images/";
    std::string output_dir = get_current_dir() + "/output_images/";
    std::string kerneldir = get_current_dir() + "/kernels/copy.cl";

    // Set image filename : default is "lenna.png", options are "image1.png" and "image2.png"
    // Image filename can be set during executation : "./read_image image1.png"
    const std::string infile = (argc > 1) ? input_dir + argv[1] : input_dir + "image4.png";
    const std::string outfile = (argc > 1) ? output_dir+ argv[1] : output_dir + "image4.png";

    // const std::string infile = input_dir + "image1.png";
    // const std::string outfile = output_dir + "image1.png";
    // std::cout << infile;

    // Create Loader for input image
    ImageMat input_img(infile);
    std::cout << "created image\n";
    // Locally store width and height of image for use in code
    const unsigned int width = input_img.width, height = input_img.height;

    // OpenCL Input Image Buffer : READ_ONLY image
    const cl::ImageFormat format(CL_RGBA, CL_UNSIGNED_INT8);
    std::cout << "set image format\n";
    cl_int error;
    cl::Image2D i_img(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, format, width, height, 0, &input_img.pixels[0], &error);
    std::cout << "created image2d\n";
    assert(error == CL_SUCCESS);
    // OpenCL Output Image Buffer : WRITE_ONLY Image
    cl::Image2D o_img(context, CL_MEM_WRITE_ONLY, format, width, height, 0, NULL);
    assert(error == CL_SUCCESS);

    // Read kernel
    cl::Program::Sources sources;
    std::string kernelcode = readFile(kerneldir);
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

    // Create command queue with current context
    cl::CommandQueue queue(context, device, 0, NULL);
    // Create kernel and set arguments
    cl::Kernel copy_kernel(program, "copy", &error);
    assert(error == CL_SUCCESS);
    copy_kernel.setArg(0, i_img);
    copy_kernel.setArg(1, o_img);

    // Execute kernel
    // Allocated memory will be same as input image dimensions because the kernel will run k^2 times each iteration if scaling_factor is k
    queue.enqueueNDRangeKernel(copy_kernel, cl::NullRange, cl::NDRange(width, height), cl::NullRange);

    // Wait for kernel code execution
    queue.finish();
    queue.flush();

    // Image reading coordinates
    cl::size_t<3> origin;
    cl::size_t<3> region;
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
    region[0] = width;
    region[1] = height;
    region[2] = 1;

    // // Output image
    ImageMat output_image;
    output_image.CreateImage(width, height);
    // Mapping image
    unsigned char * tmp = static_cast<unsigned char *>(queue.enqueueMapImage(o_img, CL_TRUE, CL_MAP_WRITE, origin, region, 0, 0, NULL, NULL, &error));
    assert(error == CL_SUCCESS);
    error = queue.enqueueUnmapMemObject(o_img, tmp, NULL, NULL);
    assert(error == CL_SUCCESS);
    
    // Copy data over from temp array to output png of same name
    std::copy(&tmp[0], &tmp[width*height*4], std::back_inserter(output_image.pixels));
    // for (int i=0; i<width*height*4; i++) {
    //     std::cout << (float)tmp[i] << "\n";
    // }
    // // Write output image to file
    std::cout << output_image.SaveImage(outfile) << std::endl;;
    // // Free image resources
    output_image.FreeImage();
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Program Execution Time (ms) : " << duration.count() << std::endl;
    return 0;
}