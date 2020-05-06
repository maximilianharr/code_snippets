# Makefile

CPP = g++ -g --std=c++14
CC  = gcc -g
INCS = -I"../include"

all:	eigen_lib kml_gnss_parser libusb_gps receive_gps reader writer recvRawEth sendRawEth cplusplus_tutorial std_vec cpp_ptr ubuntu_system

eigen_lib:		eigen_lib.cpp
			$(CPP) -o eigen_lib.out eigen_lib.cpp $(INCS)

kml_gnss_parser: 	kml_gnss_parser.cpp
			$(CPP) -o kml_gnss_parser.out kml_gnss_parser.cpp $(INCS)

libusb_gps:		libusb_gps.cpp
			$(CPP) -o libusb_gps.out libusb_gps.cpp -lusb $(INCS)

receive_gps:		receive_gps.cpp
			$(CPP) -o receive_gps.out receive_gps.cpp -lusb $(INCS)

reader:			socket_can_reader.cpp
			$(CPP) -o reader.out socket_can_reader.cpp $(INCS)

writer:			socket_can_writer.cpp
			$(CPP) -o writer.out socket_can_writer.cpp $(INCS)

recvRawEth:		recvRawEth.c
			$(CPP) -o recvRawEth.out recvRawEth.c $(INCS)

sendRawEth:		sendRawEth.c
			$(CC) -o sendRawEth.out sendRawEth.c $(INCS)

cplusplus_tutorial:	cplusplus_tutorial.cpp
			$(CPP) -o cplusplus_tutorial.out cplusplus_tutorial.cpp cplusplus_class.cpp cplusplus_class2.cpp $(INCS)

std_vec:		std_vec.cpp
			$(CPP) -o std_vec.out std_vec.cpp $(INCS)

cpp_ptr:		cpp_ptr.cpp
			$(CPP) -o cpp_ptr.out cpp_ptr.cpp $(INCS)

ubuntu_system:		ubuntu_system.cpp
			$(CPP) -o ubuntu_system.out ubuntu_system.cpp $(INCS)

# When "make clean" is triggered, then the following commands are executed:
clean:
	rm -f *.o *.out
