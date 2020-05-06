/*
    auswahl.c - verschiedene Auswahlfelder erzeugen
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>

void quit_proc(Panel_item item, Event *event)
 {
  xv_destroy_safe(xv_get(item, PANEL_CLIENT_DATA));
  return;
 }

void choice_proc(Panel_item item, int value,
                 Event *event)
 {
  Frame frame;
  static char string[100];

  frame = xv_get(item, PANEL_CLIENT_DATA);
  sprintf(string, "PANEL_VALUE=%d", value);
  xv_set(frame, FRAME_LEFT_FOOTER, string, 0);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL,       "Auswahlfelder",
    FRAME_SHOW_FOOTER, TRUE,
    0);

  panel = xv_create(frame, PANEL,
    PANEL_LAYOUT, PANEL_VERTICAL,
    0);

  xv_create(panel, PANEL_BUTTON,
    PANEL_LABEL_STRING, "Beenden",
    PANEL_NOTIFY_PROC,  quit_proc,
    PANEL_CLIENT_DATA,  frame,
    0);

  xv_create(panel, PANEL_CHOICE,
    PANEL_LABEL_STRING,   "Auswahl1:",
    PANEL_CHOICE_STRINGS, "Eins", "Zwei", NULL,
    PANEL_NOTIFY_PROC,    choice_proc,
    PANEL_CLIENT_DATA,    frame,
    0);
    
  xv_create(panel, PANEL_CHOICE,
    PANEL_CHOOSE_ONE,     FALSE,
    PANEL_LABEL_STRING,   "Auswahl2:",
    PANEL_CHOICE_STRINGS, "Eins", "Zwei", NULL,
    PANEL_NOTIFY_PROC,    choice_proc,
    PANEL_CLIENT_DATA,    frame,
    0);
    
  xv_create(panel, PANEL_CHOICE,
    PANEL_FEEDBACK,       PANEL_MARKED,
    PANEL_LABEL_STRING,   "Auswahl3:",
    PANEL_CHOICE_STRINGS, "Eins", "Zwei", NULL,
    PANEL_NOTIFY_PROC,    choice_proc,
    PANEL_CLIENT_DATA,    frame,
    0);
    
  xv_create(panel, PANEL_CHOICE,
    PANEL_DISPLAY_LEVEL,  PANEL_CURRENT,
    PANEL_LABEL_STRING,   "Auswahl4:",
    PANEL_CHOICE_STRINGS, "Eins", "Zwei", NULL,
    PANEL_NOTIFY_PROC,    choice_proc,
    PANEL_CLIENT_DATA,    frame,
    0);
    
  xv_create(panel, PANEL_CHOICE,
    PANEL_CHOOSE_ONE,     FALSE,
    PANEL_FEEDBACK,       PANEL_MARKED,
    PANEL_LABEL_STRING,   "Auswahl5:",
    PANEL_CHOICE_STRINGS, "nur ein/aus", NULL,
    PANEL_NOTIFY_PROC,    choice_proc,
    PANEL_CLIENT_DATA,    frame,
    0);
    
  window_fit(panel);
  window_fit(frame);
  xv_main_loop(frame);
  return(0);
 }
