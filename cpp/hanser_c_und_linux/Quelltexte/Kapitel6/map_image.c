/*
    map_image.c - Ein Bild mit Hilfe von 'mmap()' lesen
*/

# include <stdio.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/ioctl.h>
# include <sys/mman.h>
# include <linux/videodev.h>

# define WIDTH 320
# define HEIGHT 240
# define SWAP_RGB_BGR 0  /* 1 = Farbreihenfolge drehen */

int main()
 {
  int fd, frame, i, tmp;
  unsigned char *image;
  struct video_mmap vid_mmap;

  if ((fd = open("/dev/video", O_RDONLY)) == -1)
   {
    perror("map_image: Can't open device");
    return(1);
   }

  if ((image = mmap(NULL, WIDTH*HEIGHT*3, PROT_READ,
       MAP_SHARED, fd, 0)) == MAP_FAILED)
   {
    perror("map_image: mmap() failed");
    return(1);
   }

  vid_mmap.frame = 0;
  vid_mmap.width = WIDTH;
  vid_mmap.height = HEIGHT;
  vid_mmap.format = VIDEO_PALETTE_RGB24;
  if (ioctl(fd, VIDIOCMCAPTURE, &vid_mmap) == -1)
   {
    perror("map_image: Can't capture to memory");
    return(1);
   }

  frame = 0;	/* muss gleich vid_mmap.frame sein */
  if (ioctl(fd, VIDIOCSYNC, &frame) == -1)
   {
    perror("map_image: Can't grab frame");
    return(1);
   }
  close(fd);

  if (SWAP_RGB_BGR)	/* Rot und Blau tauschen? */
   {
    for (i=0; i<WIDTH*HEIGHT*3; i+=3)
     {
      tmp = image[i];
      image[i] = image[i+2];
      image[i+2] = tmp;
     }
   }

  printf("P6\n%d %d\n255\n", WIDTH, HEIGHT);
  fwrite(image, 3, WIDTH*HEIGHT, stdout);

  munmap(image, WIDTH*HEIGHT*3);
  return(0);
 }
