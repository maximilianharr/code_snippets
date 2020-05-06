# Makefile

CPP     = g++ -g
CC      = gcc -g
PCPP    = protoc --cpp_out=./bin
PCC     = protoc-c --c_out=./bin

INCS     = -I"./bin" 
C_INCS   = -I"./protobuf-c"
LIBS     = -lprotobuf -L /usr/local/lib
LIBS_C   = -lprotobuf-c


all:	loc_data_c loc_data loc_c loc amessage_c opelmsg_c opelmsg prot_read prot_write send_proto_udp recv_proto_udp recv_steering_udp prot_c_ex

# Make *.proto files.
# Note: protobuf-compiler has to be installed! 
# Install: sudo apt-get install protobuf-compiler protobuf-c-compiler

loc_data_c:	loc_data.proto
		$(PCC) loc_data.proto

loc_data:	loc_data.proto
		$(PCPP) loc_data.proto

loc_c:		loc.proto
		$(PCC) loc.proto

loc:		loc_data.proto
		$(PCPP) loc.proto

amessage_c:	amessage.proto
		$(PCC) amessage.proto

opelmsg_c:	opelmsg.proto
		$(PCC) opelmsg.proto

opelmsg:	opelmsg.proto
		$(PCC) opelmsg.proto

# Compile Cpp files using the protobuf-library
# Note that the protobuf library has to be installed.
# You can build libprotobuf and libprotoc as DLLs, although static linking is strongly recommended [https://github.com/google/protobuf/blob/master/cmake/README.md]
# Install: sudo apt-get install protobuf-compiler
# Check:   protoc --version

prot_read: 	prot_read.cpp
		$(CPP) prot_read.cpp ./bin/loc_data.pb.cc -o ./bin/prot_read.out $(LIBS) $(INCS)

prot_write: 	prot_write.cpp
		$(CPP) prot_write.cpp ./bin/loc_data.pb.cc -o ./bin/prot_write.out $(LIBS) $(INCS) 


# Compile C files without installing the protobuf compiler, no static linking required. It can be build from source...
# Note that protobuf-c.c has to be provided

send_proto_udp:	send_proto_udp.c
		$(CC) send_proto_udp.c ./bin/loc.pb-c.c ./protobuf-c/protobuf-c/protobuf-c.c -o ./bin/send_proto_udp.o $(INCS) $(C_INCS)

recv_proto_udp:	recv_proto_udp.c
		$(CC) recv_proto_udp.c ./bin/loc.pb-c.c ./protobuf-c/protobuf-c/protobuf-c.c -o ./bin/recv_proto_udp.o $(INCS) $(C_INCS)

# ... but can also build with dynamic linking.
# Note that now, the protobuf library has to be installed.
# Install: sudo apt-get install protobuf-c-compiler

prot_c_ex:	prot_c_ex.c
		$(CC) prot_c_ex.c ./bin/amessage.pb-c.c -o ./bin/prot_c_ex.o $(LIBS_C) $(INCS) $(C_INCS)

clean:			
		rm -rf *.o *.out *~ *.pb* ./bin/* ./build/*
