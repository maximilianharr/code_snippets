/*
    callback2.c - Funktionen beim Schließen
                  des Fensters einstellen
*/

# include <stdio.h>
# include <gtk/gtk.h>

	/*----- Callback-Funktion -----*/

gint close_win(GtkWidget *widget, GdkEvent *event,
               gpointer data)
 {
  printf("'delete'-Event ausgelöst.\n");
  return(FALSE);  /* destroy-Event auslösen */
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window;

  gtk_init(&argc, &argv);  /* Optionen auswerten */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(close_win), NULL);

  gtk_signal_connect(GTK_OBJECT(window), "destroy",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);

  gtk_widget_show(window);

  gtk_main();

  return(0);
 }
