/**
 *  @file udp_recv_proto.c
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 15.10.2016
 *
 *  @brief Receives protobuf message via UDP on specified port. [Server]
 *
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *          https://google.github.io/styleguide/cppguide.html
 *
 *
 *  @bug
 *
 *
 *  @todo
 *
 *
 */

// PRAGMA
#define BUFLEN 512  //Max length of buffer
#define PORT 40002  //The port on which to listen for incoming data

// SYSTEM INCLUDES
#include<stdio.h>
#include<stdint.h>
#include<winsock2.h>

// PROJECT INCLUDES

// LOCAL INCLUDES
#include<protobuf_header/loc.pb-c.h> /* make sure to include -I"./external" in the Makefile */

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

/** @brief
 *  @param
 *  @return
 */

// GLOBAL VARIABLES

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main()
{
	/* Initialize variables */
    SOCKET s;
    struct sockaddr_in server, si_other;
    int slen;
	size_t recv_len;
    uint8_t buf[BUFLEN];
    WSADATA wsa;
 
    slen = sizeof(si_other) ;
     
    /* Initialise winsock */
    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0) {
        printf("Failed. Error Code : %d",WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    printf("Initialised.\n");
     
    /* Create a socket */
    if((s = socket(AF_INET , SOCK_DGRAM , 0 )) == INVALID_SOCKET) {
        printf("Could not create socket : %d" , WSAGetLastError());
    }
    printf("Socket created.\n");
     
    /* Prepare the sockaddr_in structure */
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( PORT );
     
    /* Bind socket */
    if( bind(s ,(struct sockaddr *)&server , sizeof(server)) == SOCKET_ERROR) {
        printf("Bind failed with error code : %d" , WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    puts("Bind done");
 
    /* Init protobuf variables */
    Loc *msg;
    
    /* keep listening for data */
    while(1)
    {
        printf("Waiting for data...");
        fflush(stdout);
         
        /* clear the buffer by filling null, it might have previously received data */
        memset(buf,'\0', BUFLEN);
         
        /* try to receive some data, this is a blocking call */
        if ((recv_len = recvfrom(s, (char*)buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR) {
            printf("recvfrom() failed with error code : %d" , WSAGetLastError());
            exit(EXIT_FAILURE);
        }
        else{         
	        /* print details of the client/peer and the data received */
	        printf("Received packet from %s:%d with length: %i Bytes \n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port), recv_len);
	        printf("Binary UDP-packet data: %s\n" , buf);
	        
	        int i;
	        for(i = 0; i < recv_len; i++){
	        	printf("%i | ", buf[i]);
			}
			printf("\n");
	        
	        /* Show protobuf data */
	        const uint8_t* buffer = buf;
		    msg = loc__unpack(NULL, recv_len, buffer);
	    	if (msg == NULL)  {
	      	  fprintf(stderr, "error unpacking incoming message\n");
	    	}
	    	else{
	    		printf(" x=%i \n y=%i \n \n",msg->x, msg->y);
	    	}
    	}
    }
 
 	/* Close socket and cleanup */
    closesocket(s);
    WSACleanup();
    
    return 0;
}

//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


