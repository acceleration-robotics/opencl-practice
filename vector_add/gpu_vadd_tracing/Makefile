# the compiler: gcc for C program, define as g++ for C++
CC = gcc
CXX = g++

# compiler flags:
#  -g     - this flag adds debugging information to the executable file
#  -Wall  - this flag is used to turn on most compiler warnings
CFLAGS  = -g -Wall
CXXFLAGS = -llttng-ust -ldl -lOpenCL
IDIRS = .
IFLAGS = -I$(IDIRS)

# The build target 
TP_SOURCE = gpu_vadd_tp
APP_SOURCE = gpu_vadd

all: $(TP_SOURCE) $(APP_SOURCE)
	 $(CXX) -o $(APP_SOURCE) $(APP_SOURCE).o $(TP_SOURCE).o $(CXXFLAGS)

$(TP_SOURCE): $(TP_SOURCE).c
			$(CC) $(IFLAGS) $(CFLAGS) -o $(TP_SOURCE).o -c $(TP_SOURCE).c

$(APP_SOURCE) : $(APP_SOURCE).cpp
			$(CXX) -o $(APP_SOURCE).o -c $(APP_SOURCE).cpp

clean:	
		$(RM) $(TP_SOURCE).o $(APP_SOURCE).o $(APP_SOURCE)

# gcc -I. -c tpp.c
# gcc -c app.c
# gcc -o app app.o tpp.o -llttng-ust -ldl
