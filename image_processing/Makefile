
CC = gcc
CXX = g++

CFLAGS  = -g -Wall
CXXFLAGS = -llttng-ust -ldl -lOpenCL -W -Wall -Wextra -ansi -pedantic -O3
IDIRS = ./include
IFLAGS = -I$(IDIRS)
OUTIMGS = ./output_images/*

PNG_SOURCE = lodepng
# APP_SOURCE = read_image
# APP_SOURCE = resize_image
# APP_SOURCE = cpu_resize_image
# CPU_SOURCE = cpu_resize_image
CPU_SOURCE = cpu_filter_image
# OCL_SOURCE = resize_image
# OCL_SOURCE = resize_image_map
OCL_SOURCE = filter_image
# OCL_SOURCE = read_image_map

all: $(PNG_SOURCE) $(CPU_SOURCE) $(OCL_SOURCE)
	 $(CXX) -o $(CPU_SOURCE) $(CPU_SOURCE).o $(PNG_SOURCE).o $(CXXFLAGS)
	 $(CXX) -o $(OCL_SOURCE) $(OCL_SOURCE).o $(PNG_SOURCE).o $(CXXFLAGS)

$(PNG_SOURCE) : src/$(PNG_SOURCE).cpp
			$(CXX) $(IFLAGS) -o $(PNG_SOURCE).o -c src/$(PNG_SOURCE).cpp

$(CPU_SOURCE) : src/$(CPU_SOURCE).cpp
			$(CXX) $(IFLAGS) -o $(CPU_SOURCE).o -c src/$(CPU_SOURCE).cpp

$(OCL_SOURCE) : src/$(OCL_SOURCE).cpp
			$(CXX) $(IFLAGS) -o $(OCL_SOURCE).o -c src/$(OCL_SOURCE).cpp

clean:	
		$(RM) $(PNG_SOURCE).o $(OCL_SOURCE).o $(OCL_SOURCE) $(CPU_SOURCE).o $(CPU_SOURCE) $(OUTIMGS)

