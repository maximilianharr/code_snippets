/*
    xvdraw.c - Linien, Ellipsen und Rechtecke
               zeichnen
*/

# include <xview/frame.h>
# include <xview/canvas.h>
# include <xview/xv_xrect.h>

# define KEY_DRAWING 1
# define KEY_FRAME 2
# define MAX_OBJECTS 100

typedef struct
 {
  enum {line, ellipse, rect} type;
  int x1, y1, x2, y2;
 } object;

typedef struct
 {
  int num_objects;
  object object[MAX_OBJECTS];
 } drawing;

void draw_object(Display *display, Window xwin,
  GC gc, object *obj)
 {
  int x1, y1, x2, y2, x0, y0, dx, dy;

  x1 = obj->x1;
  y1 = obj->y1;
  x2 = obj->x2;
  y2 = obj->y2;
  if ((x1 == x2) && (y1 == y2))
    return;

  if (obj->type == line)
    XDrawLine(display, xwin, gc, x1, y1, x2, y2);
  else
   {
    x0 = MIN(x1, x2);
    y0 = MIN(y1, y2);
    dx = abs(x2-x1);
    dy = abs(y2-y1);
    if (obj->type == ellipse)
      XDrawArc(display, xwin, gc, x0, y0, dx, dy,
      0, 64*360);
    else
      XDrawRectangle(display, xwin, gc, x0, y0,
        dx, dy);
   }
  return;
 }

void repaint_proc(Canvas canvas, Xv_Window paint_win,
  Display *display, Window xwin, Xv_xrectlist *areas)
 {
  int i;
  drawing *drwg;
  GC gc;

  gc = DefaultGC(display, DefaultScreen(display));
  drwg = (drawing *)xv_get(paint_win,
         XV_KEY_DATA, KEY_DRAWING);
  for (i=0; i<drwg->num_objects; i++)
   draw_object(display, xwin, gc, &(drwg->object[i]));
  return;
 }

void event_proc(Xv_Window win, Event *event)
 {
  int n;
  drawing *drwg;
  object *obj;
  Frame frame;
  Display *display;
  Window xwin;
  GC gc;

  display = (Display *)xv_get(win, XV_DISPLAY);
  xwin = xv_get(win, XV_XID);
  frame = xv_get(win, XV_KEY_DATA, KEY_FRAME);
  drwg = (drawing *)xv_get(win,
              XV_KEY_DATA, KEY_DRAWING);
  n = drwg->num_objects;
  obj = &(drwg->object[n]);

  if (n == MAX_OBJECTS)
   {
    xv_set(frame, FRAME_LEFT_FOOTER,
      "maximale Anzahl von Objekten erreicht!",
      0);
    if (event_is_ascii(event)
        && (event_action(event) == 'q'))
      xv_destroy_safe(frame);
    return;
   }

  if (event_is_ascii(event))
   {
    switch(event_action(event))
     {
      case 'q': xv_destroy_safe(frame);
        return;
      case 'l': obj->type = line;
        xv_set(frame, FRAME_LEFT_FOOTER, "Linie", 0);
	break;
      case 'e': obj->type = ellipse;
        xv_set(frame, FRAME_LEFT_FOOTER,"Ellipse", 0);
	break;
      case 'r': obj->type = rect;
        xv_set(frame, FRAME_LEFT_FOOTER,"Rechteck",0);
	break;
      case ' ': drwg->num_objects = 0;
        drwg->object[0].type = obj->type;
        XClearWindow(display, xwin);
	break;
     }
    obj->x1 = obj->x2;
    obj->y1 = obj->y2;
   }
  else	/* Maustaste oder -bewegung */
   {
    gc = DefaultGC(display, DefaultScreen(display));
    if (event_action(event) == ACTION_SELECT)
     {
      if (event_is_down(event))
       {
        obj->x1 = event_x(event);
        obj->x2 = event_x(event);
        obj->y1 = event_y(event);
        obj->y2 = event_y(event);
       }
      else
       {
        XSetFunction(display, gc, GXcopy);
	draw_object(display, xwin, gc, obj);
	if (n < MAX_OBJECTS-1)
	  drwg->object[n+1].type = obj->type;
        drwg->num_objects++;
       }
     }
    else if (event_action(event) == LOC_DRAG)
     {
      XSetFunction(display, gc, GXinvert);
      draw_object(display, xwin, gc, obj);
      obj->x2 = event_x(event);
      obj->y2 = event_y(event);
      draw_object(display, xwin, gc, obj);
     }
   }
  return;
 }

/*- - - - - - - - - - - - - - - - - - - - - - - -*/

int main(int argc, char *argv[])
 {
  drawing drwg;
  Frame frame;
  Canvas canvas;

  drwg.num_objects = 0;
  drwg.object[0].type = line;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL,       argv[0],
    XV_WIDTH,          400,
    XV_HEIGHT,         300,
    FRAME_SHOW_FOOTER, TRUE,
    FRAME_LEFT_FOOTER, "Linie",
    0);

  canvas = xv_create(frame, CANVAS,
    CANVAS_REPAINT_PROC,   repaint_proc,
    CANVAS_X_PAINT_WINDOW, TRUE,
    0);

  xv_set(canvas_paint_window(canvas),
    WIN_EVENT_PROC,      event_proc,
    WIN_CONSUME_EVENTS,
      WIN_MOUSE_BUTTONS,
      LOC_DRAG,
      WIN_ASCII_EVENTS,
      0,
    XV_KEY_DATA,         KEY_DRAWING, &drwg,
    XV_KEY_DATA,         KEY_FRAME, frame,
    0);

  xv_main_loop(frame);
  return(0);
 }
