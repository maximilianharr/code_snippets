/*
    selection.c - verschiedene Auswahlelemente
*/

# include <stdio.h>
# include <string.h>
# include <gtk/gtk.h>

	/*----- Callback-Funktionen -----*/

void quit_proc(GtkWidget *widget, gpointer data)
 {
  gtk_main_quit();	/* gtk_main() beenden */
  return;
 }

void button_proc(GtkWidget *widget, gpointer data)
 {
  if (GTK_TOGGLE_BUTTON(widget)->active)
    printf("%s-Button: An\n", (char *)data);
  else
    printf("%s-Button: Aus\n", (char *)data);
  return;
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window, *vbox, *button, *separator;
  GSList *group;

  gtk_init(&argc, &argv);

	/* Fenster erzeugen */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);
  gtk_container_set_border_width(GTK_CONTAINER(window),
                                 10);

	/* vertikale Box erzeugen */

  vbox = gtk_vbox_new(FALSE, 10);
  gtk_container_add(GTK_CONTAINER(window), vbox);

	/* Toggle- und Check-Schaltflächen erzeugen */

  button = gtk_toggle_button_new_with_label("An/Aus");
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
		     GTK_SIGNAL_FUNC(button_proc),
		     "Toggle");
  gtk_box_pack_start(GTK_BOX(vbox), button,
		     FALSE, FALSE, 0);

  button = gtk_check_button_new_with_label("An/Aus");
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
		     GTK_SIGNAL_FUNC(button_proc),
		     "Check");
  gtk_box_pack_start(GTK_BOX(vbox), button,
		     FALSE, FALSE, 0);

	/* Radio-Buttons erzeugen */

  button = gtk_radio_button_new_with_label(NULL,
                                           "Auswahl 1");
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
		     GTK_SIGNAL_FUNC(button_proc),
		     "Radio1");
  gtk_box_pack_start(GTK_BOX(vbox), button,
		     FALSE, FALSE, 0);

  group = gtk_radio_button_group(GTK_RADIO_BUTTON(button));

  button = gtk_radio_button_new_with_label(group,
                                           "Auswahl 2");
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
		     GTK_SIGNAL_FUNC(button_proc),
		     "Radio2");
  gtk_box_pack_start(GTK_BOX(vbox), button,
		     FALSE, FALSE, 0);

	/* Trennlinie einfügen */

  separator = gtk_hseparator_new();
  gtk_box_pack_start(GTK_BOX(vbox), separator,
		     FALSE, FALSE, 0);

	/* Schaltfläche zum Beenden */

  button = gtk_button_new_with_label(" Beenden ");
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
                     GTK_SIGNAL_FUNC(quit_proc), NULL);
  gtk_box_pack_start(GTK_BOX(vbox), button, FALSE, FALSE,
                     0);

	/* Objekte darstellen */

  gtk_widget_show_all(window);

  gtk_main();

  return(0);
 }
