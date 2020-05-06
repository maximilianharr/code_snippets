/*
    read_image_jpeg.c - WebCam-Bild als JPEG speichern
*/

# include <stdio.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/ioctl.h>
# include <linux/videodev.h>
# include <jpeglib.h>

# define MAX_BYTES (640*480*3)	/* Bildspeicher */

# define DEF_WIDTH 320   /* Default-Werte */
# define DEF_HEIGHT 240
# define JPEG_QUALITY 75

# define SWAP_RGB_BGR 0  /* 1 = Farbreihenfolge drehen */

int main()
 {
  int fd;
  long length;
  struct video_window video_win;
  static unsigned char image[MAX_BYTES];
  JSAMPROW row_pointer;
  static struct jpeg_compress_struct cinfo; 
  struct jpeg_error_mgr jerr; 

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
    video_win.width = DEF_WIDTH;
    video_win.height = DEF_HEIGHT;
    length = DEF_WIDTH * DEF_HEIGHT * 3;
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

	/* ----- JPEG-Umwandlung vorbereiten -----*/

  cinfo.err = jpeg_std_error(&jerr); 
  jpeg_create_compress(&cinfo); 

  jpeg_stdio_dest(&cinfo, stdout); 

  cinfo.image_width = video_win.width;
  cinfo.image_height = video_win.height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, JPEG_QUALITY, TRUE);

	/* ----- JPEG-Umwandlung starten -----*/

  jpeg_start_compress(&cinfo, TRUE);

  while (cinfo.next_scanline < cinfo.image_height)
   {
    row_pointer =
      &(image[cinfo.next_scanline*cinfo.image_width*3]);
    jpeg_write_scanlines(&cinfo, &row_pointer, 1);
   }

  jpeg_finish_compress(&cinfo);   /* Bild speichern */

  jpeg_destroy_compress(&cinfo);  /* aufräumen */

  return(0);
 }
