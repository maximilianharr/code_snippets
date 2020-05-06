/*
    view_xpm_mask.c - XPM-Grafikdatei anzeigen
*/

# include <stdio.h>
# include <string.h>
# include <gtk/gtk.h>

int main(int argc, char *argv[])
 {
  GtkWidget *window, *gtk_pm;
  GdkPixmap *pixmap;
  GdkBitmap *mask;
  GtkStyle *style;
  int width, height;

	/* Kommandozeile auswerten */

  gtk_init(&argc, &argv);

  if ((argc != 2) || (strcmp(argv[1], "-h") == 0))
   {
    printf("Usage: view_xpm XPM-file\n");
    return(1);
   }

	/* Fenster erzeugen */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);
  gtk_window_set_default_size(GTK_WINDOW(window), 80, 40);
  gtk_window_set_title(GTK_WINDOW(window), argv[1]);
  gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
  gtk_widget_show(window); /* VOR gdk_pixmap_create_... */

	/* Pixmap aus XPM-Datei laden */

  style = gtk_widget_get_style(window);
  pixmap = gdk_pixmap_create_from_xpm(window->window,
				      &mask,
				      &style->white,
				      argv[1]);
  if (pixmap == NULL)
   {
    perror("view_xpm: Can't load gtk_pm");
    return(1);
   }

  gdk_window_get_size(pixmap, &width, &height);
  printf("Image size: %d x %d\n", width, height);

  gtk_pm = gtk_pixmap_new(pixmap, mask);
  gtk_container_add(GTK_CONTAINER(window), gtk_pm);

	/* Fensterhintergrund durchsichtig */

  gdk_window_shape_combine_mask(window->window, mask,
				0, 0);

	/* Fenster darstellen */

  gtk_widget_show_all(window);

  gtk_main();

  return(0);
 }
