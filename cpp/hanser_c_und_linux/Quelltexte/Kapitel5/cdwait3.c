/*
    cdwait3.c - Auf das Einlegen einer CD-ROM warten.
*/

# include <stdio.h>
# include <unistd.h>
# include <stdlib.h>
# include <fcntl.h>
# include <sched.h>
# include <signal.h>

void my_handler(int signum)
 {
  fprintf(stderr, "cdwait3: timeout\n");
  exit(1);
 }

int main()
 {
  int fd;

  signal(SIGALRM, my_handler);	/* Signal-Handler
				   einrichten */
  alarm(10);			/* Timeout: 10 Sek. */

  while ((fd = open("/dev/cdrom", O_RDONLY)) == -1)
    sched_yield();

  close(fd);
  return(0);
 }
