/*
    panel.c - Fenster mit Bedienfeld
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;
  Panel_message_item message1, message2;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    XV_WIDTH,  200,
    XV_HEIGHT, 100,
    0);

  panel = xv_create(frame, PANEL, 0);

  message1 = xv_create(panel, PANEL_MESSAGE,
    PANEL_LABEL_STRING, "Hier eine Nachricht.",
    0);

  message2 = xv_create(panel, PANEL_MESSAGE,
    PANEL_LABEL_STRING, "Und hier eine 2. Nachricht",
    0);

  xv_main_loop(frame);
  return(0);
 }
