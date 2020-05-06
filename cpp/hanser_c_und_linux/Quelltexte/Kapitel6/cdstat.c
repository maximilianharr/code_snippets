/*
    cdstat.c - Status des CD-Laufwerks abfragen
*/

# include <stdio.h>
# include <unistd.h>
# include <sys/ioctl.h>
# include <fcntl.h>
# include <linux/cdrom.h>

int main()
 {
  int fd;
  char *status_string;
  struct cdrom_subchnl subch;

  if ((fd = open("/dev/cdrom", O_RDONLY | O_NONBLOCK))
      == -1)
   {
    perror("cdstat: Can't open /dev/cdrom");
    return(1);
   }

  subch.cdsc_format = CDROM_MSF;
  if (ioctl(fd, CDROMSUBCHNL, &subch) == -1)
   {
    perror("cdstat: ioctl() failed");
    return(1);
   }
  switch(subch.cdsc_audiostatus)
   {
    case CDROM_AUDIO_PLAY:
      status_string = "playing"; break;
    case CDROM_AUDIO_PAUSED:
      status_string = "paused"; break;
    case CDROM_AUDIO_COMPLETED:
      status_string = "completed"; break;
    case CDROM_AUDIO_ERROR:
      status_string = "error"; break;
    default: status_string = "---";
   }
  printf("CD status:\t\t%s\n", status_string);
  printf("current track:\t\t%d\n", subch.cdsc_trk);
  printf("current position:\t%02d:%02d\n",
         subch.cdsc_absaddr.msf.minute,
         subch.cdsc_absaddr.msf.second);

  close(fd);
  return(0);
 }
