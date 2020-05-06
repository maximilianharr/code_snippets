/*
    canvas2.c - Zeichenflaeche mit Rollbalken
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/canvas.h>
# include <xview/scrollbar.h>
# include <xview/xv_xrect.h>

void repaint_proc(Canvas canvas, Xv_Window paint_win,
  Display *display, Window xwin, Xv_xrectlist *areas)
 {
  int i;
  GC gc;

  gc = DefaultGC(display, DefaultScreen(display));

  XClearWindow(display, xwin);

  for (i=0; i<=80; i++)
    XDrawLine(display, xwin, gc, i*10, 0, 800-i*10,
              800);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Canvas canvas;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL, argv[0],
    XV_WIDTH,    400,
    XV_HEIGHT,   300,
    0);

  canvas = xv_create(frame, CANVAS,
    CANVAS_WIDTH,          800,
    CANVAS_HEIGHT,         800,
    CANVAS_AUTO_SHRINK,    FALSE,
    CANVAS_AUTO_EXPAND,    FALSE,
    CANVAS_REPAINT_PROC,   repaint_proc,
    CANVAS_X_PAINT_WINDOW, TRUE,
    0);

  xv_create(canvas, SCROLLBAR,
    SCROLLBAR_DIRECTION, SCROLLBAR_HORIZONTAL,
    0);

  xv_create(canvas, SCROLLBAR,
    SCROLLBAR_DIRECTION, SCROLLBAR_VERTICAL,
    0);

  xv_main_loop(frame);
  return(0);
 }
