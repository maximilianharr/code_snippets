/*
    modem.c - Funktionen zur Ansteuerung des Modems
*/

# include <errno.h>
# include <string.h>
# include <unistd.h>
# include <fcntl.h>
# include <termios.h>
# include <sys/select.h>
# include <sys/time.h>

# define MODEM_DEV "/dev/modem"
# define SPEED B19200

int modem_fd = -1;  /* Datei-Deskriptor d. Modems */

int open_modem()
 {
  if ((modem_fd = open(MODEM_DEV, O_RDWR)) == -1)
    return(errno);
  else
    return(0);
 }

void close_modem()
 {
  if (modem_fd != -1)
   {
    close(modem_fd);
    modem_fd = -1;
   }
  return;
 }

int reset_modem()
 {
  struct termios term_attr;
  fd_set fdset;
  struct timeval timeout;

			/* RS232 konfigurieren */
  if (tcgetattr(modem_fd, &term_attr) != 0)
    return(errno);
  term_attr.c_cflag = CS8 | CLOCAL | CREAD;
  term_attr.c_iflag = 0;
  term_attr.c_oflag = 0;
  term_attr.c_lflag = 0;
  cfsetospeed(&term_attr, SPEED);
  cfsetispeed(&term_attr, SPEED);
  if (tcsetattr(modem_fd, TCSAFLUSH, &term_attr) != 0)
    return(errno);
    			/* Modem testen */
  if (write(modem_fd, "\r\n", 2) != 2)
    return(errno);
  usleep(200000);
  if (write(modem_fd, "ATZ\r\n", 5) != 5)
    return(errno);
  usleep(500000);

  FD_ZERO(&fdset);
  FD_SET(modem_fd, &fdset);

  timeout.tv_sec = 2;	/* 2 Sek. Timeout */
  timeout.tv_usec = 0;

			/* Antwort vom Modem? */
  if (select(modem_fd+1, &fdset, NULL, NULL,
             &timeout) <= 0)
    return(EBUSY);

  term_attr.c_cflag |= CRTSCTS;
  if (tcsetattr(modem_fd, TCSAFLUSH, &term_attr)
      != 0)
    return(errno);

  if (write(modem_fd, "ATS7=2\r\n", 8) != 8)
    return(errno);
  usleep(200000);

  return(0);
 }

int dial(char *number)
 {
  int l;

  if (write(modem_fd, "ATDT ", 5) != 5)
    return(errno);
  l = strlen(number);
  if (write(modem_fd, number, l) != l)
    return(errno);
  if (write(modem_fd, "\r\n", 2) != 2)
    return(errno);
  return(0);
 }
