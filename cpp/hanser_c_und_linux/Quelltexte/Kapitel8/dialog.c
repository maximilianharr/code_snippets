/*
    dialog.c - Dialogfenster beim Beenden öffnen
*/

# include <gtk/gtk.h>

	/*----- Callback-Funktion -----*/

gint close_win(GtkWidget *widget, GdkEvent *event,
               gpointer dialog)
 {
  gtk_widget_show(dialog);
  return(TRUE);  /* destroy-Event NICHT auslösen */
 }

gint close_sub(GtkWidget *widget, GdkEvent *event,
               gpointer data)
 {
  gtk_widget_hide(widget);
  return(TRUE);  /* destroy-Event NICHT auslösen */
 }

void cont_proc(GtkWidget *widget, gpointer subwin)
 {
  gtk_widget_hide(subwin);  /* Dialog schließen */
  return;
 }

void quit_proc(GtkWidget *widget, gpointer subwin)
 {
  gtk_widget_show(subwin);  /* Dialog öffnen */
  return;
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window, *subwin, *label, *button;

  gtk_init(&argc, &argv);  /* Optionen auswerten */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_container_set_border_width(GTK_CONTAINER(window),
                                 30);

	/* Dialogfenster erzeugen */

  subwin = gtk_dialog_new();
  gtk_signal_connect(GTK_OBJECT(subwin), "delete_event",
                     GTK_SIGNAL_FUNC(close_sub), NULL);
  gtk_window_set_title(GTK_WINDOW(subwin), "Beenden?");

	/* Callback für Hauptfenster einstellen */

  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(close_win), subwin);
  gtk_signal_connect(GTK_OBJECT(window), "destroy",
                    GTK_SIGNAL_FUNC(gtk_main_quit), NULL);

	/* Text in das Dialogfenster schreiben */

  label = gtk_label_new("Wollen Sie das Programm\n"
			"wirklich beenden?");
  gtk_box_pack_start(GTK_BOX(GTK_DIALOG(subwin)->vbox),
		     label, TRUE, TRUE, 20);
  gtk_widget_show(label);

	/* Schaltflächen für Dialog erzeugen */

  button = gtk_button_new_with_label(" Beenden ");
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
                     GTK_SIGNAL_FUNC(gtk_main_quit),
		     NULL);
  gtk_box_pack_start(
                 GTK_BOX(GTK_DIALOG(subwin)->action_area),
		 button, TRUE, FALSE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label(" Abbrechen ");
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
                     GTK_SIGNAL_FUNC(cont_proc), subwin);
  gtk_box_pack_start(
                 GTK_BOX(GTK_DIALOG(subwin)->action_area),
		 button, TRUE, FALSE, 0);
  gtk_widget_show(button);

	/* Schaltfläche für Hauptfenster erzeugen */

  button = gtk_button_new_with_label(" Beenden ");
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
                     GTK_SIGNAL_FUNC(quit_proc), subwin);
  gtk_container_add(GTK_CONTAINER(window), button);

	/* Fenster darstellen und Callback starten */

  gtk_widget_show_all(window);
  gtk_main();
  return(0);
 }
