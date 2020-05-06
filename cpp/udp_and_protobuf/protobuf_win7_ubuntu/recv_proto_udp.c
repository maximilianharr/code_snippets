/**
 *  @file recv_proto_udp.c
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 13.10.2016
 *
 *  @brief Receive protobuf file via UDP Broadcast
 *         See "C und Linux" (Hanser)
 *
 *         sudo ./recv_proto_udp.o 30000
 *
 *         Coding Standard:
 *         wiki.ros.org/CppStyleGuide
 *         https://google.github.io/styleguide/cppguide.html
 *  @bug
 *
 *
 *  @todo
 *
 *
 */

// PRAGMA
#define BUF_SIZE 1000
#define MAX_MSG_SIZE 1000

// SYSTEM INCLUDES
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// PROJECT INCLUDES

// LOCAL INCLUDES
#include "loc.pb-c.h" /* Protobuf header file */

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

/** @brief
 *  @param
 *  @return
 */

// GLOBAL VARIABLES

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
 {
  /* Init variables */
  int sock_fd, port, err, length, stop;
  struct sockaddr_in my_addr, client_addr;
  socklen_t addr_size;
  static char buffer[BUF_SIZE];

  /* Get and check input parameters */
  if ((argc != 2) || (strcmp(argv[1], "-h") == 0))  {
    fprintf(stderr, "Usage: udp-server port\n");
    return(1);
  }
  if (sscanf(argv[1], "%d", &port) != 1) {
    fprintf(stderr, "udp-server: Bad port number '%s'.\n", argv[1]);
    return(1);
  }

  /* Open UDP-socket */
  sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sock_fd == -1){
    perror("udp-server: Can't create new socket");
    return(1);
  }

  /* Bind socket to port */
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(port);
  my_addr.sin_addr.s_addr = INADDR_ANY;
  err = bind(sock_fd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr_in));
  if (err == -1) {
    perror("udp-server: bind() failed");
    return(1);
  }

  /* Init protobuf variables */
  Loc *msg;
  uint8_t buf[MAX_MSG_SIZE];

  /* Receive UDP-packets */
  stop = 0;
  while (1) {
    addr_size = sizeof(struct sockaddr_in);
    length = recvfrom(sock_fd, buf, BUF_SIZE-1, 0,
          (struct sockaddr *)&client_addr,
          &addr_size);
    if (length == -1)
      perror("udp-server: recvfrom() failed");

    else {
      buf[length] = '\0';
      printf("  TCP string with size %i byte:\n", length);
      int i,a;
      for (i = 0; i<length; i++){
        a = (int) buf[i];
        printf("    hex:%02x | int:%i\n", a,a);
      }
      printf("\n");
    }

    /* Show protobuf data */
    msg = loc__unpack(NULL, length, buf);
    if (msg == NULL)  {
      printf("Error unpacking incoming message.\n");
    }
    printf("  Protobuf-msg:\n    a=%lf \n     b=%f \n     c=%i \n     d=%li \n     e=%u \n    f=%lu \n"
                                "g=%i \n     h=%li \n     i=%u \n     j=%lu \n     k=%u \n     l=%s \n  \n", 
              msg->a, msg->b, msg->c, msg->d, msg->e, msg->f, msg->g, msg->h, msg->i, msg->j, msg->k, msg->l);
   char * b_binary = (char*) &(msg->f);
   printf("Binary Data: %x %x %x %x %x %x %x %x", (int) b_binary[0], (int) b_binary[1], (int) b_binary[2], (int) b_binary[3], (int) b_binary[4], (int) b_binary[5], (int) b_binary[6], (int) b_binary[7]);
   }
  close(sock_fd);
  return(0);
 }


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////
