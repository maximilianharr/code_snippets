/*
    input.c - verschiedene Eingabefelder
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

void text_proc(GtkWidget *widget, GdkEvent *event,
               gpointer data)
 {
  char *text;

  text = (char *)gtk_editable_get_chars(
	 GTK_EDITABLE(widget), 0,
	 gtk_text_get_length(GTK_TEXT(widget)));
  printf("Textbox:\n%s\n", text);
  return;
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window, *vbox, *button, *entry, *spin, *text;
  GtkAdjustment *adj;

  gtk_init(&argc, &argv);  /* Optionen auswerten */

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
  gtk_entry_set_text(GTK_ENTRY(entry), "Textfeld 1");
  gtk_box_pack_start(GTK_BOX(vbox), entry, FALSE, FALSE,
                     0);

  entry = gtk_entry_new_with_max_length(20);
  gtk_signal_connect(GTK_OBJECT(entry), "changed",
                     GTK_SIGNAL_FUNC(entry_proc), "2");
  gtk_entry_set_text(GTK_ENTRY(entry), "max. 20 Zeichen");
  gtk_box_pack_start(GTK_BOX(vbox), entry, FALSE, FALSE,
                     0);

	/* nummerisches Feld erzeugen */

  adj = (GtkAdjustment *)gtk_adjustment_new(20, 0, 100,
					    1, 10, 0);
  spin = gtk_spin_button_new(adj, 0.0, 0);
  gtk_signal_connect(GTK_OBJECT(adj), "value_changed",
                      GTK_SIGNAL_FUNC(spin_proc), spin);
  gtk_box_pack_start(GTK_BOX(vbox), spin, FALSE, FALSE,
                     0);

	/* mehrzeiliges Textfeld erzeugen */

  text = gtk_text_new(NULL, NULL);
  gtk_signal_connect(GTK_OBJECT(text), "focus_out_event",
                     GTK_SIGNAL_FUNC(text_proc), NULL);
  gtk_text_insert(GTK_TEXT(text), NULL, NULL, NULL,
                  "mehrzeiliger Text\n", 18);
  gtk_text_set_editable(GTK_TEXT(text), TRUE);
  gtk_text_set_word_wrap(GTK_TEXT(text), TRUE);
  gtk_box_pack_start(GTK_BOX(vbox), text, TRUE, TRUE, 0);

	/* Schaltfläche zum Beenden */

  button = gtk_button_new_with_label(" Beenden ");
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
                     GTK_SIGNAL_FUNC(quit_proc), NULL);
  gtk_box_pack_start(GTK_BOX(vbox), button, FALSE, FALSE,
                     0);

	/* Objekte darstellen */

  gtk_widget_show_all(window);

	/* Callback-Schleife starten */

  gtk_main();

  return(0);
 }
