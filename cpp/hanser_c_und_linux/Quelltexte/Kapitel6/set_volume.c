/*
    set_volume.c - Gesamtlautstaerke lesen u. schreiben
*/

# include <stdio.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/ioctl.h>
# include <linux/soundcard.h>

int main()
 {
  int fd, level;

  if ((fd = open("/dev/mixer", O_RDONLY)) == -1)
   {
    perror("set_volume: Can't open device");
    return(1);
   }

  if (ioctl(fd, MIXER_READ(SOUND_MIXER_VOLUME),
      &level) == -1)
    perror("set_volume: Can't read master volume");
  else
    printf("master volume: L=%d, R=%d\n", level & 255,
           level >> 8);

  level = 50 + (50 << 8);
  if (ioctl(fd, MIXER_WRITE(SOUND_MIXER_VOLUME),
      &level) == -1)
    perror("set_volume: Can't set master volume");

  close(fd);
  return(0);
 }
