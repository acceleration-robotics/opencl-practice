// #define CL_HPP_TARGET_OPENCL_VERSION 200
#include "utils.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

int main(int argc, char *argv[]) {
    int scaling_factor;
    std::cout << "Enter scaling factor : \n";
    std::cin >> scaling_factor;
    auto start = std::chrono::high_resolution_clock::now();

    // Set input, output image dirs and kernel dir
    std::string input_dir = get_current_dir() + "/input_images/";
    std::string output_dir = get_current_dir() + "/output_images/";
    std::string kerneldir = get_current_dir() + "/kernels/resize.cl";

    // Set image filename : default is "lenna.png", options are "image1.png" and "image2.png"
    // Image filename can be set during executation : "./read_image image1.png"
    const std::string infile = argc > 1 ? input_dir + argv[1] : input_dir + "lenna.png";
    const std::string outfile = argc > 1 ? output_dir+ argv[1] : output_dir + "lenna.png";

    // Create Loader for input image
    ImageMat input_img(infile);
    // Locally store width and height of image for use in code
    const unsigned int width = input_img.width, height = input_img.height;
    const unsigned int scaled_width = int(width*scaling_factor), scaled_height = int(height*scaling_factor);

    
    ImageMat output_img(scaled_width, scaled_height);
    std::cout << "pixelsize : " << input_img.width << std::endl;
    unsigned char input_pixels[width][height][4];
    std::vector<std::vector<std::vector<unsigned char>>> output_pixels(scaled_width, std::vector<std::vector<unsigned char>>(scaled_height, std::vector<unsigned char>(4, input_pixels[0][0][0])));
    for (int i=0; i<width; i++) {
        for (int j=0; j<height; j++){
            for (int b=0; b<4; b++) {
                input_pixels[i][j][b] = input_img.pixels[(i*4*width + j*4) + b];
                // std::cout << (int)input_pixels[i][j][b] << "\n";
                // output_img.pixels.push_back(input_img.pixels[(i*4*width + j*4) + b]);
            }
        }
    }
    int x,y;
    int inpos[2], outpos[2];
    for (int i=0; i<width; i++) {
        for (int j=0; j<height; j++) {
            inpos[0] = i + 0.4955f/scaling_factor;
            inpos[1] = j + 0.4955f/scaling_factor;
            outpos[0] = i*scaling_factor;
            outpos[1] = j*scaling_factor;
            for (int k=0; k<scaling_factor; k++) {
                for (int l=0; l<scaling_factor; l++) {
                    unsigned char * pixel = input_pixels[int(inpos[0] + k/scaling_factor)][int(inpos[1] + l/scaling_factor)];
                    for (int b=0; b<4; b++) {
                        output_pixels[int(outpos[0] + k)][int(outpos[1] + l)][b] = pixel[b];
                    }
                }
            }
        }
    }

    for (int i=0; i<scaled_width; i++) {
        for (int j=0; j<scaled_height; j++){
            for (int b=0; b<4; b++) {
                output_img.pixels.push_back(output_pixels[i][j][b]);
            }
        }
    }

    output_img.SaveImage(outfile);

    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Program Execution Time (ms) : " << duration.count() << std::endl;
    return 0;
}
