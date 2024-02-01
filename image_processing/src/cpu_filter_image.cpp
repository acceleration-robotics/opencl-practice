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

    // Set input, output image dirs and kernel dir
    std::string input_dir = get_current_dir() + "/input_images/";
    std::string output_dir = get_current_dir() + "/output_images/";

    // Set image filename : default is "lenna.png", options are "image1.png" and "image2.png"
    // Image filename can be set during executation : "./read_image image1.png"
    const std::string infile = (argc > 1) ? input_dir + argv[1] : input_dir + "image4.png";
    const std::string outfile = (argc > 1) ? output_dir+ argv[1] : output_dir + "image4.png";

    // Create Loader for input image
    ImageMat input_img(infile);
    std::cout << "created image\n";
    // Locally store width and height of image for use in code
    const unsigned int width = input_img.width, height = input_img.height;

    // Output image
    ImageMat output_image;
    output_image.CreateImage(width, height);

    // float halfsize = kernelsize/2;
    
    unsigned int input_pixels[width][height][4];
    // unsigned int output_pixels[width][height][4];

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            double sum[4] = {0, 0, 0, 0};
            for (int x = 0; x < kernelsize; x++) {
                for (int y = 0; y < kernelsize; y++) {
                    for (int b=0; b<4; b++)
                    {
                        // std::cout << "in loop\n";
                        int index = ((i + x) * width * 4 + (j + y) * 4) + b;
                        int kernelIndex = y * kernelsize + x;
                        sum[b] += input_img.pixels[index] * filter[kernelIndex];
                        // std::cout << "filter " << gaussian_filter[kernelIndex] << "\n";
                    }
                }
            }
            for (int b=0; b<4; b++)
            {
                output_image.pixels.push_back(sum[b]);
            }
        }
    }

    // Write output image to file
    std::cout << output_image.SaveImage(outfile) << std::endl;;

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Program Execution Time (ms) : " << duration.count() << std::endl;

    
    // // Free image resources
    output_image.FreeImage();
    return 0;
}