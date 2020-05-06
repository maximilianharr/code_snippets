/*
    draw.c - Grafikausgabe mit GTK
*/

# include <gtk/gtk.h>

	/*----- Callback-Funktionen -----*/

gint configure(GtkWidget *widget,
	       GdkEventConfigure *event, gpointer *pixmap)
 {
  gint width, height, depth;
  GdkGC *gc;
  GdkColor color;
  GdkColormap *colormap;
  GdkFont *font;

  if (*pixmap != NULL)
    gdk_pixmap_unref(*pixmap);
  width = widget->allocation.width;
  height = widget->allocation.height;
  depth = -1;  /* Farbtiefe auf default */
  *pixmap = gdk_pixmap_new(widget->window, width, height,
			   depth);
  gdk_draw_rectangle(*pixmap, widget->style->white_gc,
                     TRUE, 0, 0, width, height);
  gdk_draw_rectangle(*pixmap, widget->style->black_gc,
		     FALSE, 0, 0, width-1, height-1);

	/* neuer GraphicContext (GC) für Farben */

  gc = gdk_gc_new(widget->window);
  colormap = gdk_window_get_colormap(widget->window);

	/* Vordergrundfarbe orange */

  color.red = 0xf000;
  color.green = 0xb000;
  color.blue = 0x0000;
  gdk_color_alloc(colormap, &color);
  gdk_gc_set_foreground(gc, &color);

	/* 3/4-Kreis zeichnen */

  gdk_draw_arc(*pixmap, gc, TRUE, 20, 20, 100, 80,
	       0*64, 270*64);

	/* Vordergrundfarbe blaugrau */

  color.red = 0x6000;
  color.green = 0x9000;
  color.blue = 0xb000;
  gdk_color_alloc(colormap, &color);
  gdk_gc_set_foreground(gc, &color);

	/* Text ausgeben */

  font = gdk_font_load(
             "-*-times-bold-i-*-*-24-*-*-*-*-*-*-*");
  gdk_draw_text(*pixmap, font, gc, 80, 85,
		"Dies ist ein Test.", 18);

	/* Vordergrundfarbe dunkelgrün */

  color.red = 0x0000;
  color.green = 0xa000;
  color.blue = 0x0000;
  gdk_color_alloc(colormap, &color);
  gdk_gc_set_foreground(gc, &color);

	/* Dreieck zeichnen */

  gdk_gc_set_line_attributes(gc, 7, GDK_LINE_SOLID,
			  GDK_CAP_ROUND, GDK_JOIN_ROUND);
  gdk_draw_line(*pixmap, gc, 20, height-20,
		width-20, height-20);
  gdk_draw_line(*pixmap, gc, 20, height-20,
		width/2, height/2);
  gdk_draw_line(*pixmap, gc, width/2, height/2,
		width-20, height-20);

  gdk_gc_destroy(gc);
  return(TRUE);
 }

gint redraw(GtkWidget *widget, GdkEventExpose *event,
            gpointer *pixmap)
 {
  gdk_draw_pixmap(widget->window,
                  widget->style->white_gc,
                  *pixmap,
                  event->area.x, event->area.y,
                  event->area.x, event->area.y,
                  event->area.width, event->area.height);
  return(FALSE);
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window, *drawing;
  GdkPixmap *pixmap;

  gtk_init(&argc, &argv);

	/* Fenster erzeugen */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);

  pixmap = NULL;

	/* Zeichenfläche erzeugen */

  drawing = gtk_drawing_area_new();
  gtk_signal_connect(GTK_OBJECT(drawing), "expose_event",
                     GTK_SIGNAL_FUNC(redraw), &pixmap);
  gtk_signal_connect(GTK_OBJECT(drawing),
		     "configure_event",
                     GTK_SIGNAL_FUNC(configure), &pixmap);
  gtk_drawing_area_size(GTK_DRAWING_AREA(drawing),
			300, 200);
  gtk_container_add(GTK_CONTAINER(window), drawing);

	/* Objekte darstellen */

  gtk_widget_show_all(window);

  gtk_main();

  return(0);
 }
