/*
    http_service.c - HTTP-Server für WebCam
*/

# include <stdio.h>
# include <string.h>
# include <unistd.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <sys/stat.h>
# include <signal.h>
# include "usbcam.h"

# define START_FILE "start.html"
# define TMP_PATH "./tmp_"
# define BUFSIZE 2000

	/*--------------- get_line() ---------------*/

int get_line(int sock_fd, char *buffer, int length)
 {
  int i;

  i = 0;
  while ((i < length-1)
         && (recv(sock_fd, &(buffer[i]), 1, 0) == 1))
    if (buffer[i] == '\n')
      break;
    else
      i++;
  if ((i > 0) && (buffer[i-1] == '\r'))
    i--;
  buffer[i] = '\0';
  return(i);
 }

	/*--------------- file_size() ---------------*/

size_t file_size(char *filename)
 {
  struct stat file_info;

  if (stat(filename, &file_info) == -1)
    return(0);
  return(file_info.st_size);
 }

	/*-------------- send_video() --------------*/

void send_video(int client_fd, int cam_fd, char *buffer,
         int bufsize, char *cmd, int swap_RGB, int delay)
 {
  int length;
  FILE *stream;
  static char tmp_name[32];

  send(client_fd, "HTTP/1.1 200 OK\r\n"
       "Content-type: multipart/x-mixed-replace;"
       "boundary=next-jpeg-image-data\r\n\r\n", 90, 0);

  if (strcmp(cmd, "HEAD") == 0)
    return;

  snprintf(tmp_name, 31, "%s%d.jpg", TMP_PATH,
	   getpid());

  while (1)	/* Schleife, bis Verbindung abbricht */
   {
    if (get_image(cam_fd, tmp_name, swap_RGB))
      break;
    if ((stream = fopen(tmp_name, "r")) == NULL)
      break;
    sprintf(buffer, "--next-jpeg-image-data\r\n"
	    "Content-type: image/jpeg\r\n"
	    "Content-length: %ld\r\n\r\n",
	    file_size(tmp_name));
    if (send(client_fd, buffer, strlen(buffer), 0) < 0)
     {
      fclose(stream);
      break;
     }
    while (!feof(stream))
     {
      length = fread(buffer, 1, bufsize, stream);
      if (length > 0)
	if (send(client_fd, buffer, length, 0) <= 0)
	 {
	  fclose(stream);
	  remove(tmp_name);
	  return;
	 }
     }
    fclose(stream);
    send(client_fd, "\r\n", 2, 0);  /* 'Part' Ende */
    usleep(delay*1000L);
   }
  remove(tmp_name);
  return;
 }

	/*-------------- http_service() --------------*/

int http_service(int client_fd, int cam_fd, int delay,
                 int swap_RGB, char *auth)
 {
  static char buffer[BUFSIZE], cmd[8], url[256];
  char *filename;
  int length, auth_ok;
  FILE *stream;

  auth_ok = (auth[0] == '\0');  /* mit Passwort? */

  if (get_line(client_fd, buffer, BUFSIZE) == 0)
    return(0);

  if (sscanf(buffer, "%7s %255s", cmd, url) < 2)
    return(0);

	/*---- Kopfzeilen auswerten -----*/

  while (get_line(client_fd, buffer, BUFSIZE) > 0)
    if (strncasecmp(buffer, "Authorization:", 14) == 0)
      if ((!auth_ok) && (strncmp(&(buffer[21]), auth,
                         strlen(auth)) == 0))
        auth_ok = 1;

  if ((strcmp(cmd, "GET") != 0)
      && (strcmp(cmd, "HEAD") != 0))
    return(0);

	/*---- Authentifizierung -----*/

  if (!auth_ok)		/* Zugangsdaten OK? */
   {
    strcpy(buffer,
           "HTTP/1.1 401 Authorization Required\r\n"
	   "WWW-Authenticate: Basic realm=\"WebCam\"\r\n"
	   "Content-Type: text/html\r\n"
	   "Content-Length: 42\r\n\r\n"
	   "<html><body>Falsche Kennung!</body></html>");
    send(client_fd, buffer, strlen(buffer), 0);
    return(1);
   }

  filename = &(url[1]);	  /* '/' am Anfang entfernen */

  if (strcasecmp(filename, "video.jpg") == 0)
   {
    send_video(client_fd, cam_fd, buffer, BUFSIZE, cmd,
               swap_RGB, delay);
    return(0);
   }

	/*---- einfache Datei übertragen -----*/

  if (strlen(filename) == 0)
    filename = START_FILE;

  if ((stream = fopen(filename, "r")) == NULL)
   {
    send(client_fd, "HTTP/1.1 404 Not Found\r\n"
	 "Content-type: text/html\r\n"
	 "Content-length: 91\r\n\r\n"
	 "<html><head><title>Error</title></head>"
	 "<body><hr><h2>File not found.</h2><hr>"
	 "</body></html>/r/n", 164, 0);
    return(1);
   }

  sprintf(buffer, "HTTP/1.1 200 OK\r\n"
                  "Content-length: %ld\r\n\r\n",
          file_size(filename));
  if (send(client_fd, buffer, strlen(buffer), 0) <= 0)
    return(0);
  if (strcmp(cmd, "GET") == 0)
    while (!feof(stream))
     {
      length = fread(buffer, 1, BUFSIZE, stream);
      if (length > 0)
        if (send(client_fd, buffer, length, 0) <= 0)
	  return(0);
     }
  fclose(stream);
  return(1);
 }
