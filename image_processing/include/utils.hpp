#define CL_HPP_TARGET_OPENCL_VERSION 200
#define CL_HPP_ENABLE_SIZE_T_COMPATIBILITY
#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif
#include "lodepng.h"
#include <CL/opencl.hpp>
#include <fstream>
#include <iostream>

std::string get_current_dir() {
    char buff[FILENAME_MAX]; //create string buffer to hold path
    GetCurrentDir( buff, FILENAME_MAX );
    std::string current_working_dir(buff);
    return current_working_dir;
}

std::string readFile(std::string kernelfile_path) {
	std::ifstream kernelfile(kernelfile_path);
	if (kernelfile.is_open()) {
        std::cout << "File open" << "\n";
    }
    // Save kernel code in string
    std::string kernelcode((std::istreambuf_iterator<char>(kernelfile)), std::istreambuf_iterator<char>());
    std::cout << kernelcode << "\n";
	return kernelcode;
}

class ImageMat
{
public:
    unsigned width, height;
	std::vector<unsigned char> pixels;
    ImageMat() {}
    ImageMat(unsigned w, unsigned h) { CreateImage(w, h); }
	ImageMat(std::string file) { LoadImage(file); }

	unsigned LoadImage(std::string file)
	{
		FreeImage();
		return lodepng::decode(this->pixels, this->width, this->height, file.c_str());
	}
	unsigned SaveImage(std::string file)
	{
		return lodepng::encode(file.c_str(), pixels, width, height);
	}
	void CreateImage(unsigned w, unsigned h)
	{
		FreeImage();
		this->width = w;
		this->height = h;
		pixels.reserve(w * h * 4);
	}
	void FreeImage()
	{
		this->width = 0;
		this->height = 0;
		this->pixels.clear();
	}

};