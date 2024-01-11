// #define CL_HPP_TARGET_OPENCL_VERSION 200
#define CL_HPP_ENABLE_SIZE_T_COMPATIBILITY
#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif
#include <CL/cl2.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cmath>

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