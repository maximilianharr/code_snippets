/*
    get_formats.c - moegliche Samplingformate abfragen
*/

# include <stdio.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/ioctl.h>
# include <linux/soundcard.h>

int main(int argc, char *argv[])
 {
  int fd, mask;
  char *dev_name = "/dev/dsp";

  if (argc > 1)
    dev_name = argv[1];
  if ((fd = open(dev_name, O_RDONLY)) == -1)
   {
    perror("get_formats: Can't open device");
    return(1);
   }

  if (ioctl(fd, SNDCTL_DSP_GETFMTS, &mask) == -1)
    perror("get_formats: Can't get supported formats");
  else
   {
    printf("Device '%s' supports:\n", dev_name);
    if (mask & AFMT_MU_LAW)
      printf("  mu-law encoding\n");
    if (mask & AFMT_A_LAW)
      printf("  a-law encoding\n");
    if (mask & AFMT_IMA_ADPCM)
      printf("  ADPCM compression\n");
    if (mask & AFMT_U8)
      printf("  unsigned 8 bit\n");
    if (mask & AFMT_S8)
      printf("  signed 8 bit\n");
    if (mask & AFMT_S16_LE)
      printf("  signed 16 bit (little endian)\n");
    if (mask & AFMT_S16_BE)
      printf("  signed 16 bit (big endian)\n");
    if (mask & AFMT_U16_LE)
      printf("  unsigned 16 bit (little endian)\n");
    if (mask & AFMT_U16_BE)
      printf("  unsigned 16 bit (big endian)\n");
    if (mask & AFMT_MPEG)
      printf("  MPEG-2-Audio encoding\n");
   }

  close(fd);
  return(0);
 }
