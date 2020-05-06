/*
    tooltips.c - Schaltfläche mit 'Tooltips'
*/

# include <stdio.h>
# include <gtk/gtk.h>

	/*----- Callback-Funktionen -----*/

gint close_win(GtkWidget *widget, GdkEvent *event,
               gpointer data)
 {
  printf("Bitte klicken Sie auf 'Beenden'.\n");
  return(TRUE);  /* Signal löschen -> kein 'destroy' */
 }

void quit_proc(GtkWidget *widget, gpointer data)
 {
  gtk_main_quit();	/* gtk_main() beenden */
  return;
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window, *button;

  gtk_init(&argc, &argv);

	/* Fenster erzeugen */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(close_win), NULL);

  gtk_container_set_border_width(GTK_CONTAINER(window),
                                 30);

	/* Schaltfläche erzeugen */

  button = gtk_button_new_with_label(" Beenden ");

  gtk_signal_connect(GTK_OBJECT(button), "clicked",
                     GTK_SIGNAL_FUNC(quit_proc), NULL);

  gtk_container_add(GTK_CONTAINER(window), button);

	/* Tooltip erzeugen */

  GtkTooltips* tooltips = gtk_tooltips_new();

  gtk_tooltips_set_tip(tooltips, button,
		       "Zum Beenden hier klicken.", NULL);

	/* Objekte darstellen */

  gtk_widget_show(button);
  gtk_widget_show(window);

  gtk_main();

  return(0);
 }
