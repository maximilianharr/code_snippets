/**
 *  @file send_proto_udp.c
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 13.10.2016
 *
 *  @brief Send protobuf file via UDP Broadcast
 *         See "C und Linux" (Hanser)
 *         See also http://lib.protobuf-c.io/gencode.html
 *         https://github.com/protobuf-c/protobuf-c/wiki/Examples
 *
 *         sudo ./send_proto_udp.o 30000 192.168.43.255 (see Bcast with ifconfig)
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
#define BUF_SIZE 1000
#define TIMEOUT 1000

// SYSTEM INCLUDES
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <math.h>
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

//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
 {
  /* Verify that the version of the library that we linked against is
   *  compatible with the version of the headers we compiled against. */

  /* Init variables */
  char const* ip_addr;            /* Broadcast IP adress. See ifconfig */
  int sock_fd, port, length, err, i;
  struct sockaddr_in server_addr, from_addr; /* see <netinet/in.h> */
  socklen_t addr_size;
  struct pollfd pollfd;
  static char buffer[BUF_SIZE];
  Loc localization_data = LOC__INIT;
  void *buf;            // Buffer to store serialized protobuf data
  unsigned len;         // Length of serialized protobuf data


  /* Get and check input parameters */
  if (argc != 3 || (strcmp(argv[1], "-h") == 0)) {
    fprintf(stderr,"Usage: send_proto_udp port ip-addr \n");
    return;
  };
  if (sscanf(argv[1], "%d", &port) != 1) {
    fprintf(stderr, "udp-client: bad port number '%s'\n",argv[1]);
    return(1);
  }
  ip_addr = argv[2];

  /* Open UDP-Socket
   * PF_INET = Protocol family
   * SOCK_DGRAM = UDP | SOCK_STREAM = TCP/IP
   * 0 = Automatically select protocol from PF_INET
   */
  sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sock_fd == -1) {
    perror("udp-client: Can't create new socket");
    return(1);
  }

  /* Explicitly allow Broadcast for UDP-messages (setsockopt() in <sys/socket.h> */
  i = 1;
  if (setsockopt(sock_fd, SOL_SOCKET, SO_BROADCAST, &i, sizeof(i)) < 0)
    perror("udp-client: Can't set BROADCAST option.");

  /* Set IP adress */
  server_addr.sin_family = AF_INET; /* Adress family: AF_INET = IP-adress */
  server_addr.sin_port = htons(port); /* Use htons() <arpa/inet.h> to correctly set adress in sockaddr_in struct */
  err = inet_aton(ip_addr, &(server_addr.sin_addr));
  if (err == 0) {
    fprintf(stderr, "udp-client: Bad IP-Address '%s'\n", ip_addr);
    return(1);
   }
 
  uint64_t val = 0;
  unsigned int microseconds = 1000000; 
  while(1){
   /* Set values of protobuf message */
   val++;
   localization_data.a = 12.345; // double
   localization_data.b = 54.321; // float
   localization_data.c = -21; // int32
   localization_data.d = -22; // int64
   localization_data.e = 23; // uint32
   localization_data.f = 24; // uint64
   localization_data.g = -25; // int32
   localization_data.h = -26; // int64
   localization_data.i = 27; // uint32
   localization_data.j = 28; // uint64
   localization_data.k = 29; // bool
   localization_data.l = "Jeremias stinkt"; // string


   /* Serialize protobuf message */
    len = loc__get_packed_size(&localization_data);
    buf = malloc(len);
    loc__pack(&localization_data,buf);

    /* Send UDP-message (use send() for SOCK_STREAM) and check if fully transmitted */
    length = sendto(sock_fd, buf, len, 0, (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
    if (length != len){
      perror("udp-client: sendto() failed");
    }
    printf("Sending package\n");
    usleep(microseconds);
  }

  close(sock_fd);
  return(0);
 }
