/*
    draw2.c - Zeichenfläche mit Rollbalken
*/

# include <gtk/gtk.h>

# define WIDTH 500	/* Größe der Zeichenfläche */
# define HEIGHT 500

	/*----- Callback-Funktionen -----*/

gint configure(GtkWidget *widget,
	       GdkEventConfigure *event, gpointer *pixmap)
 {
  gint depth, i;

  if (*pixmap != NULL)
    gdk_pixmap_unref(*pixmap);
  depth = -1;  /* Farbtiefe auf default */
  *pixmap = gdk_pixmap_new(widget->window, WIDTH, HEIGHT,
			   depth);
  gdk_draw_rectangle(*pixmap, widget->style->white_gc,
                     TRUE, 0, 0, WIDTH, HEIGHT);
  for (i=0; i<=50; i++)
    gdk_draw_line(*pixmap, widget->style->black_gc,
		  10*i, 0, WIDTH-1-10*i, HEIGHT-1);
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
  GtkWidget *window, *scr_win, *drawing;
  GdkPixmap *pixmap;

  gtk_init(&argc, &argv);

	/* Fenster erzeugen */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);
  gtk_window_set_default_size(GTK_WINDOW(window),
			      300, 200);

	/* Fenster mit Rollbalken */

  scr_win = gtk_scrolled_window_new(NULL, NULL);
  gtk_container_add(GTK_CONTAINER(window), scr_win);

	/* Zeichenfläche erzeugen */

  pixmap = NULL;

  drawing = gtk_drawing_area_new();
  gtk_signal_connect(GTK_OBJECT(drawing), "expose_event",
                     GTK_SIGNAL_FUNC(redraw), &pixmap);
  gtk_signal_connect(GTK_OBJECT(drawing),
		     "configure_event",
		     GTK_SIGNAL_FUNC(configure), &pixmap);
  gtk_drawing_area_size(GTK_DRAWING_AREA(drawing),
			500, 500);

  gtk_scrolled_window_add_with_viewport(
                   GTK_SCROLLED_WINDOW(scr_win), drawing);

	/* Objekte darstellen */

  gtk_widget_show_all(window);

  gtk_main();

  return(0);
 }
