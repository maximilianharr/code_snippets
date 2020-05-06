/*
    text.c - Eingabefelder fuer Text und Zahlen
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>

Panel_setting text_proc(Panel_item item, Event *event)
 {
  Frame frame;
  char *label;
  static char string[80];

  frame = xv_get(item, PANEL_CLIENT_DATA);
  label = (char *)xv_get(item, PANEL_LABEL_STRING);
  if (strcmp(label, "Text:") == 0)
    sprintf(string, "Text = '%s'",
      (char *)xv_get(item, PANEL_VALUE));
  else
    sprintf(string, "Zahl = %d",
      (int)xv_get(item, PANEL_VALUE));
  xv_set(frame, FRAME_LEFT_FOOTER, string, 0);
  return(PANEL_NEXT);
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL,       "Text & Zahlen",
    FRAME_SHOW_FOOTER, TRUE,
    0);

  panel = xv_create(frame, PANEL,
    PANEL_LAYOUT, PANEL_VERTICAL,
    0);

  xv_create(panel, PANEL_TEXT,
    PANEL_LABEL_STRING,        "Text:",
    PANEL_ITEM_Y,              10,
    PANEL_VALUE_DISPLAY_WIDTH, 110,
    PANEL_VALUE_STORED_LENGTH, 20,
    PANEL_NOTIFY_PROC,         text_proc,
    PANEL_CLIENT_DATA,         frame,
    0);

  xv_create(panel, PANEL_NUMERIC_TEXT,
    PANEL_LABEL_STRING,        "Zahl:",
    PANEL_VALUE_DISPLAY_WIDTH, 70,
    PANEL_MIN_VALUE,           -10,
    PANEL_MAX_VALUE,           20,
    PANEL_NOTIFY_PROC,         text_proc,
    PANEL_CLIENT_DATA,         frame,
    0);

  window_fit(panel);
  window_fit(frame);
  xv_main_loop(frame);
  return(0);
 }
