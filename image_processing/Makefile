
CC = gcc
CXX = g++

CFLAGS  = -g -Wall
CXXFLAGS = -llttng-ust -ldl -lOpenCL -W -Wall -Wextra -ansi -pedantic -O3
IDIRS = ./include
IFLAGS = -I$(IDIRS)
OUTIMGS = ./output_images/*

PNG_SOURCE = lodepng
APP_SOURCE = read_image

all: $(PNG_SOURCE) $(APP_SOURCE)
	 $(CXX) -o $(APP_SOURCE) $(APP_SOURCE).o $(PNG_SOURCE).o $(CXXFLAGS)

$(PNG_SOURCE) : src/$(PNG_SOURCE).cpp
			$(CXX) $(IFLAGS) -o $(PNG_SOURCE).o -c src/$(PNG_SOURCE).cpp

$(APP_SOURCE) : src/$(APP_SOURCE).cpp
			$(CXX) $(IFLAGS) -o $(APP_SOURCE).o -c src/$(APP_SOURCE).cpp

clean:	
		$(RM) $(PNG_SOURCE).o $(APP_SOURCE).o $(APP_SOURCE) $(OUTIMGS)

