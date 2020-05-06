/*
    boxes.c - Schaltflächen mit vbox und hbox
              anordnen
*/

# include <stdio.h>
# include <gtk/gtk.h>

	/*----- Callback-Funktionen -----*/

void quit_proc(GtkWidget *widget, gpointer data)
 {
  gtk_main_quit();	/* gtk_main() beenden */
  return;
 }

void button_proc(GtkWidget *widget, gpointer data)
 {
  printf("%s wurde betätigt.\n", (gchar *)data);
  return;
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window, *frame1, *frame2, *box1, *box2,
            *button;
  int i;
  gchar buffer[4][16];

  gtk_init(&argc, &argv);

	/* Fenster erzeugen */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);
  gtk_container_set_border_width(GTK_CONTAINER(window),
                                 10);

	/* Frame für die 1. Box erzeugen */

  frame1 = gtk_frame_new("box1 (vbox)");
  gtk_container_add(GTK_CONTAINER(window), frame1);

	/* vertikale Box erzeugen */

  box1 = gtk_vbox_new(FALSE, 10);
  gtk_container_set_border_width(GTK_CONTAINER(box1), 10);
  gtk_container_add(GTK_CONTAINER(frame1), box1);

	/* Frame für die 2. Box erzeugen */

  frame2 = gtk_frame_new("box2 (hbox)");
  gtk_box_pack_start(GTK_BOX(box1), frame2, FALSE, FALSE,
                     0);

	/* horizontale Box erzeugen */

  box2 = gtk_hbox_new(TRUE, 10);
  gtk_container_set_border_width(GTK_CONTAINER(box2), 10);
  gtk_container_add(GTK_CONTAINER(frame2), box2);

	/* Schaltflächen erzeugen */

  for (i=0; i<4; i++)
   {
    sprintf(buffer[i], " Button %d ", i+1);
    button = gtk_button_new_with_label(buffer[i]);
    gtk_signal_connect(GTK_OBJECT(button), "clicked",
                       GTK_SIGNAL_FUNC(button_proc),
		       buffer[i]);
    gtk_box_pack_start(GTK_BOX(box2), button,
                       TRUE, TRUE, 0);
   }

  button = gtk_button_new_with_label(" Beenden ");
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
                     GTK_SIGNAL_FUNC(quit_proc), NULL);
  gtk_box_pack_start(GTK_BOX(box1), button, FALSE, FALSE,
                     0);

	/* Objekte darstellen */

  gtk_widget_show_all(window);

  gtk_main();

  return(0);
 }
