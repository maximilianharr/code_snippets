/*
    file_select.c - Dateiauswahlfenster
*/

# include <stdio.h>
# include <gtk/gtk.h>

	/*----- Callback-Funktion -----*/

void file_ok_proc(GtkWidget *widget, gpointer data)
 {
  printf("Datei laden: '%s'\n",
         gtk_file_selection_get_filename(
         GTK_FILE_SELECTION(data)));
  gtk_widget_hide(data);
  return;
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window, *filesel, *button;

  gtk_init(&argc, &argv);

	/* Fenster einrichten */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);
  gtk_container_set_border_width(GTK_CONTAINER(window),
                                 30);

	/* Dateiauswahlfenster erzeugen */

  filesel = gtk_file_selection_new ("Datei laden");
  gtk_signal_connect(GTK_OBJECT(filesel), "delete_event",
                     GTK_SIGNAL_FUNC(gtk_widget_hide),
		     &filesel);
  gtk_signal_connect(
    GTK_OBJECT(GTK_FILE_SELECTION(filesel)->ok_button),
    "clicked", GTK_SIGNAL_FUNC(file_ok_proc), filesel);
  gtk_signal_connect_object(
    GTK_OBJECT(GTK_FILE_SELECTION(filesel)->cancel_button),
    "clicked", GTK_SIGNAL_FUNC(gtk_widget_hide),
    GTK_OBJECT(filesel));

	/* Schaltfläche erzeugen */

  button = gtk_button_new_with_label(" Datei laden ");
  gtk_signal_connect_object(GTK_OBJECT(button), "clicked",
    GTK_SIGNAL_FUNC(gtk_widget_show),
    GTK_OBJECT(filesel));
  gtk_container_add(GTK_CONTAINER(window), button);
  gtk_widget_show(button);

	/* Hauptfenster darstellen */

  gtk_widget_show(window);

  gtk_main();

  return(0);
 }
