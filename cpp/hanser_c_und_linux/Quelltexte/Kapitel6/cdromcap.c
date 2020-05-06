/*
    cdromcap.c - Faehigkeiten des CDROM-Laufwerks
*/

# include <stdio.h>
# include <unistd.h>
# include <sys/ioctl.h>
# include <fcntl.h>
# include <linux/cdrom.h>

int main()
 {
  int fd, caps;

  if ((fd = open("/dev/cdrom", O_RDONLY | O_NONBLOCK))
      == -1)
   {
    perror("cdromcap: Can't open /dev/cdrom");
    return(1);
   }

  if ((caps = ioctl(fd, CDROM_GET_CAPABILITY)) == -1)
   {
    perror("cdromcap: ioctl() failed");
    return(1);
   }

  printf("Drive is a CD-R: %s, CD-RW: %s, DVD: %s, "
         "DVD-R: %s.\n",
	 (caps & CDC_CD_R)? "yes" : "no",
         (caps & CDC_CD_RW)? "yes" : "no",
         (caps & CDC_DVD)? "yes" : "no",
         (caps & CDC_DVD_R)? "yes" : "no");

  printf("It can close tray: %s, lock: %s, "
         "select disc: %s.\n",
         (caps & CDC_CLOSE_TRAY)? "no" : "yes",
         (caps & CDC_LOCK)? "yes" : "no",
         (caps & CDC_SELECT_DISC)? "yes" : "no");

  close(fd);
  return(0);
 }
