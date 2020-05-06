/*
    videoinfo.c - Informationen ueber /dev/video holen
*/

# include <stdio.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/ioctl.h>
# include <linux/videodev.h>

int main()
 {
  int fd;
  struct video_capability video_cap;
  struct video_window video_win;
  struct video_picture video_pict;

  if ((fd = open("/dev/video", O_RDONLY)) == -1)
   {
    perror("videoinfo: Can't open device");
    return(1);
   }

  if (ioctl(fd, VIDIOCGCAP, &video_cap) == -1)
    perror("videoinfo: Can't get capabilities");
  else
   {
    printf("Name:\t\t'%s'\n", video_cap.name);
    printf("Minimum size:\t%d x %d\n",
           video_cap.minwidth, video_cap.minheight);
    printf("Maximum size:\t%d x %d\n",
           video_cap.maxwidth, video_cap.maxheight);
   }

  if (ioctl(fd, VIDIOCGWIN, &video_win) == -1)
    perror("videoinfo: Can't get window information");
  else
    printf("Current size:\t%d x %d\n", video_win.width,
           video_win.height);

  if (ioctl(fd, VIDIOCGPICT, &video_pict) == -1)
    perror("videoinfo: Can't get picture information");
  else
    printf("Current depth:\t%d\n", video_pict.depth);

  close(fd);
  return(0);
 }
