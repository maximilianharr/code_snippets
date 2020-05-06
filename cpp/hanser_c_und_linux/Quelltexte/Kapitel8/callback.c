/*
    callback.c - Funktionen beim Schlieﬂen
                 des Fensters einstellen
*/

# include <gtk/gtk.h>

int main(int argc, char *argv[])
 {
  GtkWidget *window;

  gtk_init(&argc, &argv);  /* Optionen auswerten */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);

  gtk_widget_show(window);

  gtk_main();	/* Verbindung zu X-Server und
		   Callback-Mechanismus starten */

  return(0);
 }
