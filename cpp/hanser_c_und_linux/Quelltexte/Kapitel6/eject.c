/*
    eject.c - CDROM-Laufwerk oeffnen
*/

# include <stdio.h>
# include <unistd.h>
# include <sys/ioctl.h>
# include <fcntl.h>
# include <linux/cdrom.h>

int main()
 {
  int fd;

  if ((fd = open("/dev/cdrom", O_RDONLY | O_NONBLOCK))
      == -1)
   {
    perror("eject: Can't open /dev/cdrom");
    return(1);
   }

  if (ioctl(fd, CDROMEJECT) == -1)
   {
    perror("eject: ioctl() failed");
    return(1);
   }

  close(fd);
  return(0);
 }
