/*
    menu.c - Programm mit Menue-Button
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>
# include <xview/openmenu.h>

# define KEY_FRAME 1

void menu_proc(Menu menu, Menu_item item)
 {
  Frame frame;
  char *item_name;

  frame = xv_get(menu, XV_KEY_DATA, KEY_FRAME);
  item_name = (char *)xv_get(item, MENU_STRING);
  xv_set(frame, FRAME_LEFT_FOOTER, item_name, 0);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;
  Menu menu;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL,       argv[0],
    FRAME_SHOW_FOOTER, TRUE,
    0);

  panel = xv_create(frame, PANEL, 0);

  menu = xv_create(0, MENU,
    MENU_STRINGS,     "Laden...", "Speichern...", NULL,
    MENU_NOTIFY_PROC, menu_proc,
    XV_KEY_DATA,      KEY_FRAME, frame,
    0);

  xv_create(panel, PANEL_BUTTON,
    PANEL_LABEL_STRING, "Datei  ",
    PANEL_ITEM_MENU,    menu,
    0);

  xv_main_loop(frame);
  return(0);
 }
