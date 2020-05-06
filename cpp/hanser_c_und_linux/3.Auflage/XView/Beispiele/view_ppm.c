/*
    view_ppm.c - Portable Pixmap Grafik darstellen
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/canvas.h>
# include <xview/xv_xrect.h>
# include <X11/Xutil.h>

# define KEY_IMAGE 1

void repaint_proc(Canvas canvas, Xv_Window paint_win,
  Display *display, Window xwin, Xv_xrectlist *areas)
 {
  GC gc;
  XImage *image;

  gc = DefaultGC(display, DefaultScreen(display));
  image = (XImage *)xv_get(canvas,
    XV_KEY_DATA, KEY_IMAGE);
  XPutImage(display, xwin, gc, image, 0, 0, 0, 0,
    image->width, image->height);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Display *display;
  Visual *visual;
  XImage *image;
  int width, height;
  FILE *stream;
  char buffer[80], *data;

  if ((argc != 2) || (strcmp(argv[1], "-h") == 0))
   {
    printf("Usage: view_ppm filename\n");
    return(1);
   }

	/*- - - - - PPM-Datei einlesen - - - - -*/

  if ((stream = fopen(argv[1], "r")) == NULL)
   {
    perror("view_ppm: Can't open file");
    return(1);
   }
  if ((fgets(buffer, 80, stream) == NULL)
      || (strcmp(buffer, "P6\n") != 0))
   {
    fprintf(stderr, "view_ppm: Bad PPM file.\n");
    return(1);
   }
  do
    if (fgets(buffer, 80, stream) == NULL)
     {
      fprintf(stderr, "view_ppm: Bad PPM file.\n");
      return(1);
     }
  while(sscanf(buffer, "%d%d", &width, &height) != 2);
  if ((fgets(buffer, 80, stream) == NULL)
      || (strcmp(buffer, "255\n") != 0))
   {
    fprintf(stderr, "view_ppm: Unsupported format\n");
    return(1);
   }

  if ((data = (void *)malloc(width*height*3)) == NULL)
   {
    fprintf(stderr, "view_ppm: Not enough memory.\n");
    return(1);
   }

  fread(data, 3, width*height, stream);
  fclose(stream);

	/*- - - - - XView-Objekte erzeugen - - - - -*/

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    XV_WIDTH,                 width+2,
    XV_HEIGHT,                height+2,
    FRAME_SHOW_RESIZE_CORNER, FALSE,
    FRAME_LABEL,              argv[1],
    0);

  display = (Display *)xv_get(frame, XV_DISPLAY);
  if (DefaultDepth(display, DefaultScreen(display))
    != 24)
   {
    fprintf(stderr, "view_ppm: Set X11 to 24 bpp.\n");
    return(1);
   }
  visual = (Visual *)xv_get(frame, XV_VISUAL);

  image = XCreateImage(display, visual, 24, ZPixmap,
    0, data, width, height, 16, width*3);
  image->byte_order = MSBFirst;
  image->bits_per_pixel = 24;	/* wichtig für Grafikkarten mit 32 Bit/Pixel! */

  xv_create(frame, CANVAS,
    CANVAS_AUTO_SHRINK,    FALSE,
    CANVAS_AUTO_EXPAND,    FALSE,
    CANVAS_REPAINT_PROC,   repaint_proc,
    CANVAS_X_PAINT_WINDOW, TRUE,
    XV_KEY_DATA,           KEY_IMAGE, image,
    0);

  xv_main_loop(frame);
  XDestroyImage(image);
  return(0);
 }
