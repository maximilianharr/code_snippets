/*
    read_image.c - Bild von WebCam oder TV-Karte lesen
*/

# include <stdio.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/ioctl.h>
# include <linux/videodev.h>

# define MAX_BYTES (640*480*3)	/* Bildspeicher */
# define SWAP_RGB_BGR 0  /* 1 = Farbreihenfolge drehen */

int main()
 {
  int fd;
  long length;
  struct video_window video_win;
  static unsigned char image[MAX_BYTES];

  if ((fd = open("/dev/video", O_RDONLY)) == -1)
   {
    perror("read_image: Can't open device");
    return(1);
   }

  if (ioctl(fd, VIDIOCGWIN, &video_win) == -1)
   {
    perror("read_image: Can't get video window");
    return(1);
   }

  length = video_win.width * video_win.height * 3;

  if ((length < 1) || (length > MAX_BYTES))
   {
    fprintf(stderr, "read_image: Bad image size. "
                    "Using default values.\n");
    video_win.width = 320;
    video_win.height = 240;
    length = 320 * 240 * 3;
    if (ioctl(fd, VIDIOCSWIN, &video_win) == -1)
     {
      perror("read_image: Can't set video window");
      return(1);
     }
   }

  if (read(fd, image, length) == -1)
   {
    perror("read_image: Error while reading");
    return(1);
   }
  close(fd);

  if (SWAP_RGB_BGR)	/* Rot und Blau tauschen? */
   {
    int i, tmp;

    for (i=0; i<video_win.width*video_win.height*3; i+=3)
     {
      tmp = image[i];
      image[i] = image[i+2];
      image[i+2] = tmp;
     }
   }

  printf("P6\n%d %d\n255\n", video_win.width,
         video_win.height);
  fwrite(image, 3, video_win.width*video_win.height,
         stdout);

  return(0);
 }
