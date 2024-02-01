#define CL_HPP_TARGET_OPENCL_VERSION 200
#include "utils.hpp"
#include <iostream>
#include <cassert>

int main(int argc, char *argv[]) {

    unsigned int kernelsize, filter_type;
    double sigma;
    std::cout << "Enter filter size (odd number) : ";
    while (!(std::cin >> kernelsize) || kernelsize % 2 == 0) {
        std::cout << "Invalid input. Please enter an odd number: ";
        std::cin.clear(); // Clear the error state
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard the input
    }
    std::cout << "Enter filter type -->\n0 : Gaussian,\n1 : Mean\n"; // ,\n2 : Median,\n3 : Sobel\n";
     while (!(std::cin >> filter_type) || filter_type <0 || filter_type > 3) {
        std::cout << "Invalid input. Please enter filter type in range 0 to 4: ";
        std::cin.clear(); // Clear the error state
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard the input
    }
    int size = int(kernelsize*kernelsize);
    float filter[size];
    if (filter_type == 0)
    
    {   
        sigma = 0.3*((kernelsize-1)*0.5 - 1) + 0.8;
        std::cout << "Filter size set to : " << kernelsize << "\n Standard deviation set to : " << sigma << "\n";
        

    double r, s = 2.0 * sigma * sigma;
    double sum = 0.0;

    int halfSize = (kernelsize - 1) / 2;
    int index = 0;

    for (int x = -halfSize; x <= halfSize; x++) {
        for (int y = -halfSize; y <= halfSize; y++) {
            std::cout << index << "\n";
            r = sqrt(x * x + y * y);
            filter[index++] = exp(-(r * r) / s) / (M_PI * s);
            sum += filter[index - 1];
        }
    }

    for (int i = 0; i < size; ++i) {
        filter[i] /= sum;
        std::cout << filter[i] << "\t";
    }
    }

    if (filter_type == 1)
    {
        for (int i=0; i<size; i++) {
            filter[i] = 1.0/(kernelsize*kernelsize);
        }
    }

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
    std::string kerneldir = get_current_dir() + "/kernels/filter.cl";

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
    // Buffer for gaussian kernel
    cl::Buffer mask(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(float)*(kernelsize*2+1)*(kernelsize*2+1), filter, &error);
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
    cl::Kernel filter_kernel(program, "filter_kernel", &error);
    assert(error == CL_SUCCESS);
    filter_kernel.setArg(0, i_img);
    filter_kernel.setArg(1, o_img);
    filter_kernel.setArg(2, mask);
    filter_kernel.setArg(3, kernelsize);

    // Execute kernel
    // Allocated memory will be same as input image dimensions because the kernel will run k^2 times each iteration if scaling_factor is k
    queue.enqueueNDRangeKernel(filter_kernel, cl::NullRange, cl::NDRange(width, height), cl::NullRange);

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
    // unsigned char * tmp = static_cast<unsigned char *>(queue.enqueueMapImage(o_img, CL_TRUE, CL_MAP_WRITE, origin, region, 0, 0, NULL, NULL, &error));
    // assert(error == CL_SUCCESS);
    // error = queue.enqueueUnmapMemObject(o_img, tmp, NULL, NULL);
    // assert(error == CL_SUCCESS);


    // Temporary buffer to store image data : has to be on the heap so kernel can access it
    auto tmp = new unsigned char[width * height * 4];
    // Blocking copy call
    error = queue.enqueueReadImage(o_img, CL_TRUE, origin, region, 0, 0, tmp);
    assert(error == CL_SUCCESS);
    // Copy data over from temp array to output png of same name
    std::copy(&tmp[0], &tmp[width*height*4], std::back_inserter(output_image.pixels));
    // Write output image to file
    std::cout << output_image.SaveImage(outfile) << std::endl;;

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Program Execution Time (ms) : " << duration.count() << std::endl;
    
    // Free image resources
    output_image.FreeImage();
    // Free temp array
    delete[] tmp;

    return 0;
}