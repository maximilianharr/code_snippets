/*
    button-grafik.c - Schaltfläche mit Grafik und Text
*/

# include <stdio.h>
# include <gtk/gtk.h>

static const char *xpm_data[] = {
	"16 14 3 1",
	"  c None",
	". c #000000000000",
	"X c #FFFFFFFFFFFF",
	"   ......       ",
	"   .XXX.X.      ",
	"   .XXX.XX.     ",
	"   .XXX.XXX.    ",
	"   .XXX.....    ",
	"   .XXXXXXX.    ",
	"   .XXXXXXX.    ",
	"   .XXXXXXX.    ",
	"   .XXXXXXX.    ",
	"   .XXXXXXX.    ",
	"   .XXXXXXX.    ",
	"   .XXXXXXX.    ",
	"   .........    ",
	"                "};

	/*----- Callback-Funktion -----*/

void button_proc(GtkWidget *widget, gpointer data)
 {
  printf("Schaltfläche 'neue Datei' betätigt.\n");
  return;
 }

	/*----- Hauptprogramm -----*/

int main(int argc, char *argv[])
 {
  GtkWidget *window, *button, *hbox, *label, *gtk_pm;
  GdkPixmap *pixmap;
  GdkBitmap *mask;

  gtk_init(&argc, &argv);

	/* Fenster erzeugen */

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);
  gtk_container_set_border_width(GTK_CONTAINER(window),
                                 30);
  gtk_widget_show(window);

	/* Schaltfläche erzeugen */

  button = gtk_button_new();
  gtk_signal_connect(GTK_OBJECT(button), "clicked",
                     GTK_SIGNAL_FUNC(button_proc), NULL);
  gtk_container_add(GTK_CONTAINER(window), button);

	/* hbox erzeugen */

  hbox = gtk_hbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(button), hbox);

	/* Pixmap erzeugen */

  pixmap = gdk_pixmap_create_from_xpm_d(window->window,
                            &mask, &window->style->white,
			    (gchar **)xpm_data);
  gtk_pm = gtk_pixmap_new(pixmap, mask);
  gtk_box_pack_start(GTK_BOX(hbox), gtk_pm, FALSE, FALSE,
		     0);

	/* label erzeugen und in hbox stellen */

  label = gtk_label_new("neue Datei");
  gtk_box_pack_start(GTK_BOX(hbox), label, FALSE, FALSE,
		     4);

	/* Objekte darstellen */

  gtk_widget_show_all(window);

  gtk_main();

  return(0);
 }
