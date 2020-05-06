/*
    upnp-search.c - UPnP-Dienste abfragen
*/

# include <stdio.h>
# include <unistd.h>
# include <string.h>
# include <sys/poll.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <sys/time.h>
# include <netinet/in.h>
# include <arpa/inet.h>

# define BUF_SIZE 20000
# define TIMEOUT 3000	/* Millisekunden */

# define UPNP_ADDR "239.255.255.250"
# define UPNP_PORT 1900

int main()
 {
  int sock_fd, length, i, err;
  struct sockaddr_in server_addr, from_addr;
  socklen_t addr_size;
  struct pollfd pollfd;
  static char buffer[BUF_SIZE];

				 /*--- Socket öffnen ---*/
  sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sock_fd == -1)
   {
    perror("upnp-search: Can't create new socket");
    return(1);
   }
				   /*--- Zieladresse ---*/
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(UPNP_PORT);
  inet_aton(UPNP_ADDR, &(server_addr.sin_addr));

			        /*--- Anfrage senden ---*/
  strcpy(buffer, "M-SEARCH * HTTP/1.1\r\n"
                 "HOST: 239.255.255.250:1900\r\n"
		 "MAN: \"ssdp:discover\"\r\n"
		 "MX: 2\r\n"
		 "ST: ssdp:all\r\n"
		 "\r\n");

  length = sendto(sock_fd, buffer, strlen(buffer), 0,
                  (struct sockaddr *)&server_addr,
		  sizeof(struct sockaddr));
  if (length != strlen(buffer))
   {
    perror("upnp-search: sendto() failed");
    return(1);
   }
			    /*--- auf Antwort warten ---*/
  pollfd.fd = sock_fd;		/* Polling vorbereiten */
  pollfd.events = POLLIN | POLLPRI;
  i = 0;
  while (1)
   {
    err = poll(&pollfd, 1, TIMEOUT);
    if (err < 0)
     {
      perror("upnp-search: poll() failed");
      break;
     }
    else if (err == 0)		/* Timeout erreicht */
     {
      if (i == 0)
        printf("<No response received.>\n");
      break;
     }
    else
     {
      addr_size = sizeof(struct sockaddr_in);
      length = recvfrom(sock_fd, buffer, BUF_SIZE-1, 0,
			(struct sockaddr *)&from_addr,
			&addr_size);
      if (length == -1)
	perror("upnp-search: recvfrom() failed");
      else
       {
        buffer[length] = '\0';
	printf("\33[1m---- Response from %s:\33[0m\n%s\n",
	       inet_ntoa(from_addr.sin_addr), buffer);
       }
      i = 1;
     }
   }
  close(sock_fd);
  return(0);
 }
