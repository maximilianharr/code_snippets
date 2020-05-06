/*
    menu2.c - Fenster mit Menüleiste und Trennlinie
*/

# include <stdio.h>
# include <string.h>
# include <gtk/gtk.h>

GtkWidget *label;  /* für Callback sichtbar! */

	/*----- Callback-Funktion -----*/

void menu_proc(GtkWidget *widget, gpointer data)
 {
  static char buffer[40];

  sprintf(buffer, "MenÃ¼punkt '%s' gewÃ¤hlt.",
	  (char *)data);
  gtk_label_set_text(GTK_LABEL(label), buffer);
  return;
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window, *vbox, *menubar, *menu, *menuitem;

  gtk_init(&argc, &argv);

	/* Fenster erzeugen */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);
  gtk_window_set_default_size(GTK_WINDOW(window),
			      250, 150);

	/* vertikale Box erzeugen */

  vbox = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), vbox);

	/* Menüs erzeugen und einrichten */

  menubar = gtk_menu_bar_new();
  gtk_box_pack_start(GTK_BOX(vbox), menubar, FALSE, FALSE,
		     2);

  menu = gtk_menu_new();

  menuitem = gtk_menu_item_new_with_label("Laden");
  gtk_signal_connect(GTK_OBJECT(menuitem), "activate",
                     GTK_SIGNAL_FUNC(menu_proc),
		     "Laden");
  gtk_menu_append(GTK_MENU(menu), menuitem);

  menuitem = gtk_menu_item_new_with_label("Speichern");
  gtk_signal_connect(GTK_OBJECT(menuitem), "activate",
                     GTK_SIGNAL_FUNC(menu_proc),
		     "Speichern");
  gtk_menu_append(GTK_MENU(menu), menuitem);

  menuitem = gtk_separator_menu_item_new();  /* Trennlinie */
  gtk_menu_append(GTK_MENU(menu), menuitem);

  menuitem = gtk_menu_item_new_with_label("Beenden");
  gtk_signal_connect(GTK_OBJECT(menuitem), "activate",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);
  gtk_menu_append(GTK_MENU(menu), menuitem);

  menuitem = gtk_menu_item_new_with_label("Datei");
  gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuitem),
			    menu);
  gtk_menu_bar_append(GTK_MENU_BAR(menubar), menuitem);

	/* Label als Leerraum erzeugen */

  label = gtk_label_new("");
  gtk_box_pack_start(GTK_BOX(vbox), label, TRUE, TRUE, 0);

	/* Label zur Textanzeige erzeugen */

  label = gtk_label_new("Kein MenÃ¼punkt gewÃ¤hlt.");
  gtk_box_pack_start(GTK_BOX(vbox), label, FALSE, FALSE,
		     0);

	/* Objekte darstellen */

  gtk_widget_show_all(window);

  gtk_main();

  return(0);
 }
