/*
    connect2.c - Netzwerk-Client mit DNS-Nutzung
*/

# include <stdio.h>
# include <unistd.h>
# include <string.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <netinet/in.h>
# include <arpa/inet.h>
# include <netdb.h>

int main(int argc, char *argv[])
 {
  static char buffer[256];
  int sock_fd, err, length, port;
  struct hostent *server;
  struct in_addr *server_ip;
  struct sockaddr_in server_addr;
  fd_set input_fdset;

  if (argc != 3)
   {
    fprintf(stderr, "Usage: connect hostname port\n");
    return(1);
   }

  if (sscanf(argv[2], "%d", &port) != 1)
   {
    fprintf(stderr, "connect: bad argument '%s'\n",
            argv[2]);
    return(1);
   }

  server = gethostbyname(argv[1]);
  if (server == NULL)
   {
    herror("connect2: Can't get IP-address");
    return(1);
   }
  server_ip = (struct in_addr *)server->h_addr;

  sock_fd = socket(PF_INET, SOCK_STREAM, 0);
  if (sock_fd == -1)
   {
    perror("connect2: Can't create new socket");
    return(1);
   }

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = server_ip->s_addr;

  err = connect(sock_fd, (struct sockaddr *)&server_addr,
                sizeof(struct sockaddr_in));
  if (err == -1)
   {
    perror("connect2: connect() failed");
    return(1);
   }

  while (1)
   {
    FD_ZERO(&input_fdset);
    FD_SET(STDIN_FILENO, &input_fdset);
    FD_SET(sock_fd, &input_fdset);
    if (select(sock_fd+1, &input_fdset, NULL, NULL,
               NULL) == -1)
      perror("connect2: select() failed");
    if (FD_ISSET(STDIN_FILENO, &input_fdset))
     {
      if (fgets(buffer, 256, stdin) == NULL)
       {
        printf("connect2: Closing socket.\n");
	break;
       }
      length = strlen(buffer);
      send(sock_fd, buffer, length, 0);
     }
    else
     {
      length = recv(sock_fd, buffer, 256, 0);
      if (length == 0)
       {
        printf("Connection closed by remote host.\n");
        break;
       }
      write(STDOUT_FILENO, buffer, length);
     }
   }

  close(sock_fd);
  return(0);
 }
