/**
 *  @file udp_server.c
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 15.10.2016
 *
 *  @brief Receive UDP-message and display.
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
#define PORT 4321   //The port on which to listen for incoming data

// SYSTEM INCLUDES
#include<stdio.h>
#include<winsock2.h>

// PROJECT INCLUDES

// LOCAL INCLUDES

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
	/* Init variables */
    SOCKET s;
    struct sockaddr_in server, si_other;
    int slen , recv_len;
    char buf[BUFLEN];
    WSADATA wsa;
 
    slen = sizeof(si_other) ;
     
    /* Initialise winsock */
    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        printf("Failed. Error Code : %d",WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    printf("Initialised.\n");
     
    /* Create a socket */
    if((s = socket(AF_INET , SOCK_DGRAM , 0 )) == INVALID_SOCKET)
    {
        printf("Could not create socket : %d" , WSAGetLastError());
    }
    printf("Socket created.\n");
     
    /* Prepare the sockaddr_in structure */
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( PORT );
     
    /* Bind */
    if( bind(s ,(struct sockaddr *)&server , sizeof(server)) == SOCKET_ERROR)
    {
        printf("Bind failed with error code : %d" , WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    puts("Bind done");
 
    /* keep listening for data */
    while(1)
    {
        printf("Waiting for data...");
        fflush(stdout);
         
        /* clear the buffer by filling null, it might have previously received data */
        memset(buf,'\0', BUFLEN);
         
        /* try to receive some data, this is a blocking call */
        if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR)
        {
            printf("recvfrom() failed with error code : %d" , WSAGetLastError());
            exit(EXIT_FAILURE);
        }
         
        /* print details of the client/peer and the data received */
        printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        printf("Data: %s\n" , buf);
         
        /* now reply the client with the same data */
        if (sendto(s, buf, recv_len, 0, (struct sockaddr*) &si_other, slen) == SOCKET_ERROR)
        {
            printf("sendto() failed with error code : %d" , WSAGetLastError());
            exit(EXIT_FAILURE);
        }
    }
 
    /* Close socket and cleanup */
    closesocket(s);
    WSACleanup();
     
    return 0;
}

//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


