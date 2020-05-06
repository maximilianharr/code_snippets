/*
    webcam.c - Webserver für USB-Kamera
		- Hauptteil -
*/

# include <stdio.h>
# include <string.h>
# include <stdlib.h>
# include <unistd.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <netinet/in.h>
# include <arpa/inet.h>
# include <sys/stat.h>
# include <signal.h>
# include "http_service.h"
# include "usbcam.h"

# define IP_PORT 80
# define N_CONNECTIONS 10

void err_exit(char *message)
 {
  perror(message);
  exit(1);
 }

/*--------------- Hauptprogramm ---------------*/

int main(int argc, char *argv[])
 {
  int option, sock_fd, client_fd, cam_fd, err, pid,
      delay, swap_RGB;
  char *dev_name, *auth;
  socklen_t addr_size;
  struct sockaddr_in my_addr, client_addr;

    /*---- Defaultwerte für die Einstellungen ----*/

  swap_RGB = 0;		      /* Rot/Blau tauschen? */
  dev_name = "/dev/video";    /* Video4Linux device */
  auth = "";	  /* Username:Passwort, base64-codiert */
  delay = 100;	  /* Wartezeit zwischen den Bildern */

    /*---- Kommandozeilenparameter auswerten ----*/

  while ((option = getopt(argc, argv, "hsa:d:")) >= 0)
    switch (option)
     {
      case 'h':
        printf("Usage: %s [-s] [-d #] [-a str] [device]\n"
               "-s     : swap colours from BGR to RGB\n"
               "-a str : set HTTP authentication code\n"
               "-d #   : set delay between images\n",
	       argv[0]);
	return(0);
      case 's': swap_RGB = 1;
		break;
      case 'a': auth = optarg;
		break;
      case 'd': sscanf(optarg, "%d", &delay);
		break;
      case '?': return(1);     /* unbekannte Option */
     }

  if (argc-optind > 1)
   {
    fprintf(stderr, "webcam: Bad arguments.\n");
    return(1);
   }

  if (argc-optind == 1)
    dev_name = argv[optind];

	/*---- Socket öffnen und an Port binden ----*/

  sock_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd == -1)
    err_exit("webcam: Can't create new socket");

  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(IP_PORT);
  my_addr.sin_addr.s_addr = INADDR_ANY;

  err = bind(sock_fd, (struct sockaddr *)&my_addr,
             sizeof(struct sockaddr_in));
  if (err == -1)
    err_exit("webcam: bind() failed");

  setuid(getuid());	/* root-Rechte abgeben */

  err = listen(sock_fd, N_CONNECTIONS);
  if (err == -1)
    err_exit("webcam: listen() failed");

	/*---- Kamera öffnen und initialisieren ----*/

  if ((cam_fd = init_cam(dev_name)) < 0)
    return(1);

  signal(SIGCHLD, SIG_IGN);  /* keine Zombie-Prozesse */

  signal(SIGPIPE, SIG_IGN);  /* wenn Browser beendet */

  printf("HTTP service started.\n"
         "Press Ctrl-C to stop.\n");

  while (1)		/*---- Endlosschleife ----*/
   {
    addr_size = sizeof(struct sockaddr_in);
    client_fd = accept(sock_fd,
      (struct sockaddr *)&client_addr, &addr_size);
    if (client_fd == -1)
      err_exit("webcam: accept() failed");

    if ((pid = fork()) == -1)
     {
      fprintf(stderr, "webcam: fork() failed.\n");
      return(1);
     }
    else if (pid == 0)		/* Kind-Prozess */
     {
      while (http_service(client_fd, cam_fd, delay,
                          swap_RGB, auth));
      shutdown(client_fd, SHUT_RDWR);
      close(client_fd);
      return(0);
     }
    close(client_fd);		/* Eltern-Prozess */
   }

  return(0);	/* wird nie erreicht */
 }
