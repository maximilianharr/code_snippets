/*
    rec_play.c - Audio-Signal aufnehmen u. wiedergeben
*/

# include <stdio.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/ioctl.h>
# include <linux/soundcard.h>

# define NUM_SAMPLES 100000

unsigned char buffer[NUM_SAMPLES];

int main()
 {
  int fd, i, format=AFMT_U8;
  long length;
  char input[16];

  if ((fd = open("/dev/dsp", O_RDWR)) == -1)
   {
    perror("rec_play: Can't open device");
    return(1);
   }

  if (ioctl(fd, SNDCTL_DSP_SETFMT, &format) == -1)
    perror("rec_play: Can't set format");

  i = 0;
  if (ioctl(fd, SNDCTL_DSP_STEREO, &i) == -1)
    perror("rec_play: Can't set to mono");

  i = 22050;
  if (ioctl(fd, SNDCTL_DSP_SPEED, &i) == -1)
    perror("rec_play: Can't set sampling rate");

  printf("Press <RETURN> to start recording. ");
  fgets(input, 16, stdin);

  if ((length = read(fd, buffer, NUM_SAMPLES)) == -1)
   {
    perror("rec_play: Can't record audio data");
    return(1);
   }

  printf("done (%ld bytes).\n"
         "Press <RETURN> to start playing. ", length);
  fgets(input, 16, stdin);

  if (write(fd, buffer, length) == -1)
    perror("rec_play: Can't play audio data");

  close(fd);
  return(0);
 }
