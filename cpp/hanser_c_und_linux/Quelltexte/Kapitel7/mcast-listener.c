/*
 * listener.c -- joins a multicast group and echoes all data it receives from
 *		the group to its stdout...
 *
 * Antony Courtney,	25/11/94
 * Modified by: Fr�d�ric Bastien (25/03/04)
 * to compile without warning and work correctly
 *
 * Source: http://ntrg.cs.tcd.ie/undergrad/4ba2/multicast/antony/example.html
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <string.h>
#include <stdio.h>


#define HELLO_PORT 1900			/* port and IP address for UPnP */
#define HELLO_GROUP "239.255.255.250"
// #define HELLO_PORT 1901
// #define HELLO_GROUP "224.0.0.37"

#define MSGBUFSIZE 256

int main(int argc, char *argv[])
{
     struct sockaddr_in addr;
     int fd, nbytes;
     socklen_t addrlen;
     struct ip_mreq mreq;
     char msgbuf[MSGBUFSIZE];

     /* create what looks like an ordinary UDP socket */
     if ((fd=socket(AF_INET,SOCK_DGRAM,0)) < 0) {
	  perror("socket");
	  return(1);
     }

/**** MODIFICATION TO ORIGINAL */
    /* allow multiple sockets to use the same PORT number */
     u_int yes = 1;
     if (setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes)) < 0) {
        perror("Reusing ADDR failed");
        return(1);
       }
/*** END OF MODIFICATION TO ORIGINAL */

     /* set up destination address */
     memset(&addr,0,sizeof(addr));
     addr.sin_family=AF_INET;
     addr.sin_addr.s_addr=htonl(INADDR_ANY); /* N.B.: differs from sender */
     addr.sin_port=htons(HELLO_PORT);
     
     /* bind to receive address */
     if (bind(fd,(struct sockaddr *) &addr,sizeof(addr)) < 0) {
	  perror("bind");
	  return(1);
     }
     
     /* use setsockopt() to request that the kernel join a multicast group */
     mreq.imr_multiaddr.s_addr=inet_addr(HELLO_GROUP);
     mreq.imr_interface.s_addr=htonl(INADDR_ANY);
     if (setsockopt(fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) < 0) {
	  perror("setsockopt");
	  return(1);
     }

     /* now just enter a read-print loop */
     while (1) {
	  addrlen=sizeof(addr);
	  if ((nbytes=recvfrom(fd,msgbuf,MSGBUFSIZE,0,
			       (struct sockaddr *) &addr,&addrlen)) < 0) {
	       perror("recvfrom");
	       return(1);
	  }
          msgbuf[nbytes] = 0;
	  puts(msgbuf);
     }
     return (0);
}

