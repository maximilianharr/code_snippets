/**
 *  @file udp_send_proto.c
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 15.10.2016
 *
 *  @brief Sends protobuf message via UDP on specified port / IP.
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
#define SERVER "127.0.0.1"  // IP address of udp server
#define PORT 40001   		//The port on which to listen for incoming data

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
int main(void)
{
    struct sockaddr_in si_other;
    int s, slen=sizeof(si_other);
    WSADATA wsa;
 
    //Initialise winsock
    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0) {
        printf("Failed. Error Code : %d",WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    printf("Initialised.\n");
     
    //create socket
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
        printf("socket() failed with error code : %d" , WSAGetLastError());
        exit(EXIT_FAILURE);
    }
     
    //setup address structure
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);
    
    /* Create protobuf object */ 
    Loc localization_data = LOC__INIT;
    void *buf;     	// Buffer to store serialized protobuf data
    unsigned len;	// Length of serialized protobuf data
    
    //start communication
    while(1) {
         
	  /* Set values of protobuf message */
	  localization_data.x = 2;
	  localization_data.y = 16;
	
	  /* Serialize protobuf message */
	  len = loc__get_packed_size(&localization_data);
	  buf = malloc(len);
	  loc__pack(&localization_data,buf);
    
      //send the message
      if (sendto(s, buf, len, 0 , (struct sockaddr *) &si_other, slen) == SOCKET_ERROR) {
          printf("sendto() failed with error code : %d" , WSAGetLastError());
          getchar();
          exit(EXIT_FAILURE);
      }
      
      printf("Sending package... \n");
      sleep(1);
    }
 
    closesocket(s);
    WSACleanup();
 
    return 0;
}

//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////

