/*
    input.c - verschiedene Text-Eingabefelder
*/

# include <stdio.h>
# include <gtk/gtk.h>

	/*----- Callback-Funktionen -----*/

void quit_proc(GtkWidget *widget, gpointer data)
 {
  gtk_main_quit();	/* gtk_main() beenden */
  return;
 }

void entry_proc(GtkWidget *widget, gpointer data)
 {
  printf("Eingabe in Textfeld %s: %s\n",
         (char *)data,
	 gtk_entry_get_text(GTK_ENTRY(widget)));
  return;
 }

void spin_proc(GtkWidget *widget, gpointer data)
 {
  printf("Eingabe in Zahlenfeld: %d\n",
         gtk_spin_button_get_value_as_int(data));
  return;
 }

gboolean text_proc(GtkWidget *view, GdkEvent *event,
		   gpointer data)
 {
  GtkTextBuffer *buffer;
  GtkTextIter start, end;
  char *text;

  buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view));
  gtk_text_buffer_get_start_iter(buffer, &start);
  gtk_text_buffer_get_end_iter(buffer, &end);
  text = gtk_text_buffer_get_text(buffer, &start, &end,
				  FALSE);
  printf("Textbox:\n%s\n", text);
  return(FALSE); /* Signal nicht löschen! */
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window, *vbox, *button, *entry, *spin, *view;
  GtkTextBuffer *buffer;
  GtkAdjustment *adj;

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

	/* Textfelder erzeugen */

  entry = gtk_entry_new();
  gtk_signal_connect(GTK_OBJECT(entry), "changed",
                     GTK_SIGNAL_FUNC(entry_proc), "1");
  gtk_box_pack_start(GTK_BOX(vbox), entry, FALSE, FALSE,
                     0);
  gtk_entry_set_text(GTK_ENTRY(entry), "Textfeld 1");

  entry = gtk_entry_new_with_max_length(20);
  gtk_signal_connect(GTK_OBJECT(entry), "changed",
                     GTK_SIGNAL_FUNC(entry_proc), "2");
  gtk_box_pack_start(GTK_BOX(vbox), entry, FALSE, FALSE,
                     0);
  gtk_entry_set_text(GTK_ENTRY(entry), "max. 20 Zeichen");

	/* nummerisches Feld erzeugen */

  adj = (GtkAdjustment *)gtk_adjustment_new(20, 0, 100,
					    1, 10, 0);
  spin = gtk_spin_button_new(adj, 0.0, 0);
  gtk_signal_connect(GTK_OBJECT(adj), "value_changed",
                      GTK_SIGNAL_FUNC(spin_proc), spin);
  gtk_box_pack_start(GTK_BOX(vbox), spin, FALSE, FALSE,
                     0);

	/* Textbox erzeugen */

  view = gtk_text_view_new();
  gtk_signal_connect(GTK_OBJECT(view), "focus_out_event",
                     GTK_SIGNAL_FUNC(text_proc), NULL);
  gtk_widget_set_usize(view, 200, 120);
  gtk_text_view_set_editable(GTK_TEXT_VIEW(view), TRUE);
  gtk_box_pack_start(GTK_BOX(vbox), view, TRUE, TRUE, 0);

  buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view));
  gtk_text_buffer_set_text(buffer, "mehrzeiliger Text\n",
			   -1);

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
