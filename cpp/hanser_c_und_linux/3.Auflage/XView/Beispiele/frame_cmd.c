/*
    frame_cmd.c - Command Frame erzeugen
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>

void save_proc(Panel_item item, Event *event)
 {
  Frame frame_cmd;

  frame_cmd = xv_get(item, PANEL_CLIENT_DATA);
  xv_set(frame_cmd,
    FRAME_CMD_PUSHPIN_IN, TRUE,
    XV_SHOW,              TRUE,
    0);
  return;
 }

void done_proc(Frame frame_cmd)
 {
  xv_set(frame_cmd, XV_SHOW, FALSE, 0);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame, frame_cmd;
  Panel panel, panel_cmd;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL, "Hauptfenster",
    0);

  panel = xv_create(frame, PANEL, 0);

  frame_cmd = xv_create(frame, FRAME_CMD,
    FRAME_LABEL,     "Datei speichern",
    FRAME_DONE_PROC, done_proc,
    XV_SHOW,         FALSE,
    0);

  panel_cmd = xv_get(frame_cmd, FRAME_CMD_PANEL);

  xv_create(panel_cmd, PANEL_TEXT,
    PANEL_LABEL_STRING,        "Dateiname",
    PANEL_VALUE_DISPLAY_WIDTH, 120,
    PANEL_ITEM_Y,              20,
    0);

  window_fit(panel_cmd);
  window_fit(frame_cmd);

  xv_create(panel, PANEL_BUTTON,
    PANEL_LABEL_STRING, "Datei speichern",
    PANEL_NOTIFY_PROC,  save_proc,
    PANEL_CLIENT_DATA,  frame_cmd,
    0);

  xv_main_loop(frame);
  return(0);
 }
