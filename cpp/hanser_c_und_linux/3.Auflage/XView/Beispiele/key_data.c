/*
    key_data.c - Daten an Callback-Funktion uebergeben
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>

# define KEY_FRAME 1
# define KEY_ARGV 2

void button1_proc(Panel_item button, Event *event)
 {
  Frame frame;
  char **argv;

  frame = xv_get(xv_get(button, XV_OWNER), XV_OWNER);
  argv = (char **)xv_get(button, PANEL_CLIENT_DATA);
  xv_set(frame,
    FRAME_LEFT_FOOTER,  "mit 'CLIENT_DATA':",
    FRAME_RIGHT_FOOTER, argv[0],
    FRAME_SHOW_FOOTER,  TRUE,
    0);
  return;
 }

void button2_proc(Panel_item button, Event *event)
 {
  Frame frame;
  char **argv;

  frame = xv_get(button, XV_KEY_DATA, KEY_FRAME);
  argv = (char **)xv_get(button,
                         XV_KEY_DATA, KEY_ARGV);
  xv_set(frame,
    FRAME_LEFT_FOOTER,  "mit 'KEY_DATA':",
    FRAME_RIGHT_FOOTER, argv[0],
    FRAME_SHOW_FOOTER,  TRUE,
    0);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME, 0);

  panel = xv_create(frame, PANEL, 0);

  xv_create(panel, PANEL_BUTTON,
    PANEL_LABEL_STRING, "mit `CLIENT_DATA`",
    PANEL_NOTIFY_PROC,  button1_proc,
    PANEL_CLIENT_DATA,  argv,
    0);

  xv_create(panel, PANEL_BUTTON,
    PANEL_LABEL_STRING, "mit `KEY_DATA`",
    PANEL_NOTIFY_PROC,  button2_proc,
    XV_KEY_DATA,        KEY_FRAME, frame,
    XV_KEY_DATA,        KEY_ARGV, argv,
    0);

  xv_main_loop(frame);
  return(0);
 }
