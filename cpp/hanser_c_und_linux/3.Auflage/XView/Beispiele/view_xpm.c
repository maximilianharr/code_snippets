/*
    view_xpm.c - X11-Pixmap darstellen
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/canvas.h>
# include <xview/xv_xrect.h>
# include <X11/xpm.h>

# define KEY_PIXMAP 1
# define KEY_WIDTH 2
# define KEY_HEIGHT 3

void repaint_proc(Canvas canvas, Xv_Window paint_win,
  Display *display, Window xwin, Xv_xrectlist *areas)
 {
  GC gc;
  Pixmap pixmap;
  int width, height;

  gc = DefaultGC(display, DefaultScreen(display));
  pixmap = xv_get(canvas, XV_KEY_DATA, KEY_PIXMAP);
  width = xv_get(canvas, XV_KEY_DATA, KEY_WIDTH);
  height = xv_get(canvas, XV_KEY_DATA, KEY_HEIGHT);
  XCopyArea(display, pixmap, xwin, gc, 0, 0,
    width, height, 0, 0);
  return;
 }

int main(int argc, char *argv[])
 {
  int width, height, error;
  Frame frame;
  Canvas canvas;
  Window xwin;
  Display *display;
  Pixmap pixmap, shape;
  XpmAttributes attributes;

  if ((argc != 2) || (strcmp(argv[1], "-h") == 0))
   {
    printf("Usage: view_xpm filename\n");
    return(1);
   }

	/*- - - - - XView-Objekte erzeugen - - - - -*/

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL,              argv[1],
    FRAME_SHOW_RESIZE_CORNER, FALSE,
    0);

  display = (Display *)xv_get(frame, XV_DISPLAY);
  xwin = xv_get(frame, XV_XID);

  attributes.valuemask = 0;
  error = XpmReadFileToPixmap(display, xwin, argv[1],
    &pixmap, &shape, &attributes);
  if (error)		/* nicht genug Farben frei? */
   {
    fprintf(stderr, "view_xpm: Can't read pixmap.\n");
    return(1);
   }

  width = attributes.width;
  height = attributes.height;

  xv_set(frame,
    XV_WIDTH,  width+2,
    XV_HEIGHT, height+2,
    0);

  canvas = xv_create(frame, CANVAS,
    CANVAS_REPAINT_PROC,   repaint_proc,
    CANVAS_X_PAINT_WINDOW, TRUE,
    XV_KEY_DATA,           KEY_PIXMAP, pixmap,
    XV_KEY_DATA,           KEY_WIDTH, width,
    XV_KEY_DATA,           KEY_HEIGHT, height,
    0);

  xv_main_loop(frame);
  XFreePixmap(display, pixmap);
  return(0);
 }
