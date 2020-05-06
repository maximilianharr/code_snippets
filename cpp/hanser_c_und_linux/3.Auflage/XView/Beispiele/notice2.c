/*
    notice2.c - Eine Warnung ausgeben
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>
# include <xview/notice.h>

# define KEY_FRAME 1
# define KEY_NOTICE 2

void quit_proc(Panel_item item, Event *event)
 {
  Frame frame;
  Xv_notice notice;
  int choice;

  frame = xv_get(item, XV_KEY_DATA, KEY_FRAME);
  notice = xv_get(item, XV_KEY_DATA, KEY_NOTICE);
  xv_set(notice,
    XV_SHOW,       TRUE,
    NOTICE_STATUS, &choice,
    0);
  if (choice == NOTICE_YES)
    xv_destroy_safe(frame);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;
  Xv_notice notice;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL, argv[0],
    0);

  panel = xv_create(frame, PANEL, 0);

  notice = xv_create(frame, NOTICE,
    NOTICE_MESSAGE_STRING, "Warnung!\n"
      "Nicht gespeicherte Daten gehen verloren!\n"
      "Wollen Sie wirklich beenden?",
    NOTICE_BUTTON_YES,     "  Ja  ",
    NOTICE_BUTTON_NO,      " Nein ",
    0);

  xv_create(panel, PANEL_BUTTON,
    PANEL_LABEL_STRING, "Beenden",
    PANEL_NOTIFY_PROC,  quit_proc,
    XV_KEY_DATA,        KEY_FRAME, frame,
    XV_KEY_DATA,        KEY_NOTICE, notice,
    0);

  xv_main_loop(frame);
  return(0);
 }
