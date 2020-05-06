/*
    canvas.c - Eine Zeichenflaeche
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/cms.h>
# include <xview/canvas.h>
# include <xview/xv_xrect.h>
# include <xview/font.h>

# define KEY_FONT 1

void repaint_proc(Canvas canvas, Xv_Window paint_win,
  Display *display, Window xwin, Xv_xrectlist *areas)
 {
  GC gc;
  XGCValues gc_values;
  Cms cms;
  int width, height;
  long red, blue;
  Xv_Font font;

  width = xv_get(paint_win, XV_WIDTH);
  height = xv_get(paint_win, XV_HEIGHT);
  cms = xv_get(canvas, WIN_CMS);
  red = xv_get(cms, CMS_PIXEL, 1);
  blue = xv_get(cms, CMS_PIXEL, 2);
  font = xv_get(canvas, XV_KEY_DATA, KEY_FONT);
  gc = XCreateGC(display, xwin, 0, &gc_values);

  XClearWindow(display, xwin);

  XDrawLine(display, xwin, gc, 10, 10, 200, 80);

  gc_values.foreground = red;
  gc_values.line_width = 3;
  XChangeGC(display, gc, GCForeground | GCLineWidth,
    &gc_values);

  XDrawArc(display, xwin, gc, 1, 1,
    width-3, height-3, 0, 360*64);

  XSetForeground(display, gc, blue);
  if (font != XV_NULL)
    XSetFont(display, gc, xv_get(font, XV_XID));

  XDrawString(display, xwin, gc, 20, 100,
    "Hier ist eine Leinwand", 22);

  XFreeGC(display, gc);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Cms cms;
  Xv_Font font;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL, argv[0],
    XV_WIDTH,    300,
    XV_HEIGHT,   200,
    0);

  cms = xv_create(0, CMS,
    CMS_SIZE,         4,
    CMS_NAMED_COLORS, "white", "red", "blue",
      "black", NULL,
    0);
  if (cms == XV_NULL)
   {
    fprintf(stderr, "color: Can't create CMS.\n");
    return(1);
   }

  font = xv_find(frame, FONT,
    FONT_FAMILY, FONT_FAMILY_GALLENT,
    FONT_STYLE,  FONT_STYLE_ITALIC,
    FONT_SIZE,   24,
    0);

  xv_create(frame, CANVAS,
    CANVAS_REPAINT_PROC,   repaint_proc,
    CANVAS_X_PAINT_WINDOW, TRUE,
    CANVAS_RETAINED,       FALSE,
    WIN_CMS,               cms,
    XV_KEY_DATA,           KEY_FONT, font,
    0);

  xv_main_loop(frame);
  return(0);
 }
