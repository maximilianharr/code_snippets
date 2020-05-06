/*
    usbcam.c - Kamera-Ansteuerung für WebCam
*/

# include <stdio.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/ioctl.h>
# include <linux/videodev.h>
# include <jpeglib.h>

# define WIDTH 320	/* Bildgröße QVGA */
# define HEIGHT 240
# define IMGSIZE (WIDTH*HEIGHT*3L)  /* Bildspeicher */
# define JPEG_QUALITY 70

	/*--------------- init_cam() ---------------*/

int init_cam(char *device_name)
 {
  int fd;
  struct video_window video_win;

  if ((fd = open(device_name, O_RDONLY)) == -1)
   {
    perror("webcam: Can't open video device");
    return(-1);
   }

  if (ioctl(fd, VIDIOCGWIN, &video_win) == -1)
   {
    perror("webcam: Can't get video window");
    return(-1);
   }

  video_win.width = WIDTH;
  video_win.height = HEIGHT;

  if (ioctl(fd, VIDIOCSWIN, &video_win) == -1)
   {
    perror("webcam: Can't set video window");
    return(-1);
   }

  return(fd);
 }

	/*--------------- get_image() ---------------*/

int get_image(int cam_fd, char *tmpfile, int swap_RGB)
 {
  int i, tmp;
  long len;
  static unsigned char image[IMGSIZE];
  FILE *stream;
  JSAMPROW row_pointer;
  static struct jpeg_compress_struct cinfo; 
  struct jpeg_error_mgr jerr; 

  len = read(cam_fd, image, IMGSIZE);
  if (len < IMGSIZE)
   {
    perror("webcam: Error while reading");
    if (len < 0)
      return(1);
   }

  if (swap_RGB)		/* Rot und Blau tauschen? */
    for (i=0; i<IMGSIZE; i+=3)
     {
      tmp = image[i];
      image[i] = image[i+2];
      image[i+2] = tmp;
     }

	/* ----- JPEG-Umwandlung vorbereiten -----*/

  cinfo.err = jpeg_std_error(&jerr); 
  jpeg_create_compress(&cinfo); 

  if ((stream = fopen(tmpfile, "w")) == NULL)
   {
    perror("webcam: Can't open temporary file.");
    return(1);
   }

  jpeg_stdio_dest(&cinfo, stream); 

  cinfo.image_width = WIDTH;
  cinfo.image_height = HEIGHT;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, JPEG_QUALITY, TRUE);

	/* ----- JPEG-Umwandlung starten -----*/

  jpeg_start_compress(&cinfo, TRUE);

  while (cinfo.next_scanline < HEIGHT)
   {
    row_pointer = &(image[cinfo.next_scanline*WIDTH*3]);
    jpeg_write_scanlines(&cinfo, &row_pointer, 1);
   }

  jpeg_finish_compress(&cinfo);   /* Bild speichern */

  jpeg_destroy_compress(&cinfo);  /* aufräumen */

  fclose(stream);
  return(0);
 }
