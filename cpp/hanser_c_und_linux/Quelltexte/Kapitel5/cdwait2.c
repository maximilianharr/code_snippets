/*
    cdwait2.c - Auf das Einlegen einer CD-ROM warten.
*/

# include <unistd.h>
# include <fcntl.h>
# include <sched.h>

int main()
 {
  int fd;

  alarm(10);	/* Timeout: 10 Sekunden */

  while ((fd = open("/dev/cdrom", O_RDONLY)) == -1)
    sched_yield();

  close(fd);
  return(0);
 }
