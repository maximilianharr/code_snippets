# Makefile
# Cmake stands for Cross-platform make.
#
# Zieldatei:	Quelldatei1, Quelldatei2, ...
# 		Instruction1
# 		Instruction2
# 		...
#
# set --std=c++14 if using C++14 revision. eg. for uniform initialization (since C++11)
# set -c for the compiler to prevent linking of files.
# set -o for the executeable file name.


connect2:		connect2.c
			gcc connect2.c -o connect2.o

connect:		connect.c
			gcc connect.c -o connect.o

ip-lookup:		ip-lookup.c
			gcc ip-lookup.c -o ip-lookup.o

mcast-listener:		mcast-listener.c
			gcc mcast-listener.c -o mcast-listener.o

server:			server.c
			gcc server.c -o server.o

udp-client2:		udp-client2.c
			gcc udp-client2.c -o udp-client2.o

udp-client:		udp-client.c
			gcc udp-client.c -o udp-client.o

udp-server:		udp-server.c
			gcc udp-server.c -o udp-server.o

upnp-search:		upnp-search.c
			gcc upnp-search.c -o upnp-search.o

webserver:		webserver.c
			gcc webserver.c -o webserver.o


# When "make clean" is triggered, then the following commands are executed:
clean:
	rm -f *.o *.out
