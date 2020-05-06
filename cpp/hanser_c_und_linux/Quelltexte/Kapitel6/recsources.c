/*
    recsources.c - moegliche Aufnahmequellen des Mixers
*/

# include <stdio.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/ioctl.h>
# include <linux/soundcard.h>

const char *device_names[SOUND_MIXER_NRDEVICES]
           = SOUND_DEVICE_NAMES;

int main()
 {
  int fd, mask, i;

  if ((fd = open("/dev/mixer", O_RDONLY)) == -1)
   {
    perror("recsources: Can't open device");
    return(1);
   }

  if (ioctl(fd, MIXER_READ(SOUND_MIXER_RECMASK), &mask)
      == -1)
    perror("recsources: ioctl() failed");
  else
   {
    printf("Supported recording sources:");
    for (i=0; i<SOUND_MIXER_NRDEVICES; i++)
      if (mask & (1<<i))
	printf(" %s,", device_names[i]);
    printf("\b.\n");
   }

  close(fd);
  return(0);
 }
