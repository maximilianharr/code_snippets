/*
    webserver.c - minimalistischer HTTP-Server
*/

# include <stdio.h>
# include <string.h>
# include <unistd.h>
# include <stdlib.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <netinet/in.h>
# include <arpa/inet.h>
# include <sys/stat.h>
# include <signal.h>

# define MY_PORT 80
# define N_CONNECTIONS 20
# define HTML_PATH "."
# define DEFAULT_FILE "index.html"

void err_exit(char *message)
 {
  perror(message);
  exit(1);
 }

int get_line(int sock_fd, char *buffer, int length)
 {
  int i;

  i = 0;
  while ((i < length-1) &&
         (recv(sock_fd, &(buffer[i]), 1, 0) == 1))
    if (buffer[i] == '\n')
      break;
    else
      i++;
  if ((i > 0) && (buffer[i-1] == '\r'))
    i--;
  buffer[i] = '\0';
  return(i);
 }

int is_html(char *filename)
 {
  if (strcmp(&(filename[strlen(filename)-5]), ".html")
       == 0)
    return(1);
  if (strcmp(&(filename[strlen(filename)-4]), ".htm")
      == 0)
    return(1);
  return(0);
 }

size_t file_size(char *filename)
 {
  struct stat file_info;

  if (stat(filename, &file_info) == -1)
    return(0);
  return(file_info.st_size);
 }

void http_service(int client_fd)
 {
  char buffer[256], cmd[8], url[128], *filename;
  int length;
  FILE *stream;

  if (get_line(client_fd, buffer, 256) == 0)
    return;
  if (sscanf(buffer, "%7s %127s", cmd, url) < 2)
    return;
  while (get_line(client_fd, buffer, 256) > 0);
  if ((strcmp(cmd, "GET") != 0)
      && (strcmp(cmd, "HEAD") != 0))
    return;

  filename = &(url[1]);
  if (strlen(filename) == 0)
    filename = DEFAULT_FILE;	

  if ((stream = fopen(filename, "r")) == NULL)
   {
    send(client_fd, "HTTP/1.0 404 Not Found\r\n"
         "Content-type: text/html\r\n"
         "Content-length: 91\r\n\r\n"
         "<html><head><title>Error</title></head>"
         "<body><hr><h2>File not found.</h2><hr>"
	 "</body></html>", 162, 0);
    return;
   }

  send(client_fd, "HTTP/1.0 200 OK\r\n", 17, 0);
  if (is_html(filename))
    send(client_fd, "Content-type: text/html\r\n", 25, 0);
  sprintf(buffer, "Content-length: %ld\r\n\r\n",
          file_size(filename));
  send(client_fd, buffer, strlen(buffer), 0);
  if (strcmp(cmd, "GET") == 0)
    while (!feof(stream))
     {
      length = fread(buffer, 1, 256, stream);
      if (length > 0)
        send(client_fd, buffer, length, 0);
     }
  fclose(stream);
  return;
 }

/*--------------- Hauptprogramm ---------------*/

int main()
 {
  int sock_fd, client_fd, err, pid;
  struct sockaddr_in my_addr, client_addr;
  socklen_t addr_size;

  sock_fd = socket(PF_INET, SOCK_STREAM, 0);
  if (sock_fd == -1)
    err_exit("webserver: Can't create new socket");

  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(MY_PORT);
  my_addr.sin_addr.s_addr = INADDR_ANY;

  err = bind(sock_fd, (struct sockaddr *)&my_addr,
             sizeof(struct sockaddr_in));
  if (err == -1)
    err_exit("webserver: bind() failed");

  setuid(getuid());

  err = listen(sock_fd, N_CONNECTIONS);
  if (err == -1)
    err_exit("webserver: listen() failed");

  if (chdir(HTML_PATH) != 0)
    err_exit("webserver: Can't set HTML path");

  signal(SIGCHLD, SIG_IGN);

  printf("Type Ctrl-C to stop.\n");

  while (1)
   {
    addr_size = sizeof(struct sockaddr_in);
    client_fd = accept(sock_fd,
            (struct sockaddr *)&client_addr, &addr_size);
    if (client_fd == -1)
      err_exit("webserver: accept() failed");

    if ((pid = fork()) == -1)
     {
      fprintf(stderr, "webserver: fork() failed.\n");
      return(1);
     }
    else if (pid == 0)		/* Kind-Prozess */
     {
      close(sock_fd);
      http_service(client_fd);
      shutdown(client_fd, SHUT_RDWR);
      close(client_fd);
      return(0);
     }
    close(client_fd);
   }

  return(0);		/* wird nie erreicht */
 }
