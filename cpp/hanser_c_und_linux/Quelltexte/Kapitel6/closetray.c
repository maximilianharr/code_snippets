/*
    closetray.c - CDROM-Laufwerk schliessen
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
    perror("closetray: Can't open /dev/cdrom");
    return(1);
   }

  if (ioctl(fd, CDROMCLOSETRAY) == -1)
   {
    perror("closetray: ioctl() failed");
    return(1);
   }

  close(fd);
  return(0);
 }
