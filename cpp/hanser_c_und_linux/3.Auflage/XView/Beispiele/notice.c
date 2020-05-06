/*
    notice.c - Eine Warnung ausgeben
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>
# include <xview/notice.h>

void quit_proc(Panel_item item, Event *event)
 {
  Frame frame;
  int choice;

  frame = xv_get(item, PANEL_CLIENT_DATA);
  choice = notice_prompt(frame, NULL,
    NOTICE_MESSAGE_STRING, "Warnung!\n"
      "Nicht gespeicherte Daten gehen verloren!\n"
      "Wollen Sie wirklich beenden?",
    NOTICE_BUTTON_YES,     "  Ja  ",
    NOTICE_BUTTON_NO,      " Nein ",
    0);
  if (choice == NOTICE_YES)
    xv_destroy_safe(frame);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL, argv[0],
    0);

  panel = xv_create(frame, PANEL, 0);

  xv_create(panel, PANEL_BUTTON,
    PANEL_LABEL_STRING, "Beenden",
    PANEL_NOTIFY_PROC,  quit_proc,
    PANEL_CLIENT_DATA,  frame,
    0);

  xv_main_loop(frame);
  return(0);
 }
