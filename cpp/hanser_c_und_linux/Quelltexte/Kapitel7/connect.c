/*
    connect.c - einfacher Netzwerk-Client
*/

# include <stdio.h>
# include <unistd.h>
# include <string.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <netinet/in.h>
# include <arpa/inet.h>

int main(int argc, char *argv[])
 {
  static char buffer[256];
  int sock_fd, err, length, port;
  struct sockaddr_in server_addr;
  fd_set input_fdset;

  if (argc != 3)
   {
    fprintf(stderr, "Usage: connect ip-addr port\n");
    return(1);
   }

  if (sscanf(argv[2], "%d", &port) != 1)
   {
    fprintf(stderr, "connect: bad argument '%s'\n",
            argv[2]);
    return(1);
   }

  sock_fd = socket(PF_INET, SOCK_STREAM, 0);
  if (sock_fd == -1)
   {
    perror("connect: Can't create new socket");
    return(1);
   }

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  err = inet_aton(argv[1], &(server_addr.sin_addr));
  if (err == 0)
   {
    fprintf(stderr, "connect: Bad IP-Address '%s'\n",
            argv[1]);
    return(1);
   }

  err = connect(sock_fd, (struct sockaddr *)&server_addr,
                sizeof(struct sockaddr_in));
  if (err == -1)
   {
    perror("connect: connect() failed");
    return(1);
   }

  while (1)
   {
    FD_ZERO(&input_fdset);
    FD_SET(STDIN_FILENO, &input_fdset);
    FD_SET(sock_fd, &input_fdset);
    if (select(sock_fd+1, &input_fdset, NULL, NULL, NULL)
        == -1)
      perror("connect: select() failed");
    if (FD_ISSET(STDIN_FILENO, &input_fdset))
     {
      if (fgets(buffer, 256, stdin) == NULL)
       {
        printf("connect: Closing socket.\n");
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
