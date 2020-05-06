/*
    quit.c - Fenster mit einer Schaltflaeche
             zum Beenden des Programms
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>

void quit_proc(Panel_item item, Event *event)
 {
  Frame frame;

  frame = xv_get(xv_get(item, XV_OWNER), XV_OWNER);
  xv_destroy_safe(frame);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;
  Panel_button_item quit_button;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    XV_WIDTH,  200,
    XV_HEIGHT, 100,
    0);

  panel = xv_create(frame, PANEL, 0);

  quit_button = xv_create(panel, PANEL_BUTTON,
    PANEL_ITEM_X,       20,
    PANEL_ITEM_Y,       50,
    PANEL_LABEL_STRING, "Programm beenden",
    PANEL_NOTIFY_PROC,  quit_proc,
    0);

  xv_main_loop(frame);
  return(0);
 }
