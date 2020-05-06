/*
    choose_file.c - Dateiauswahlfenster oeffnen
*/

# include <stdio.h>
# include <unistd.h>
# include <xview/frame.h>
# include <xview/panel.h>
# include <xview/file_chsr.h>

void quit_proc(Panel_item item, Event *event)
 {
  xv_destroy_safe(xv_get(item, PANEL_CLIENT_DATA));
  return;
 }

void choose_file_proc(Panel_item item, Event *event)
 {
  File_chooser file_chooser;

  file_chooser = xv_get(item, PANEL_CLIENT_DATA);
  xv_set(file_chooser, XV_SHOW, TRUE, 0);
  return;
 }

int load_proc(File_chooser file_ch, char *filename)
 {
  printf("Dateiname: '%s'\n", filename);
  return(XV_OK);
 }

/*- - - - - - - - - - - - - - - - - - - - - - - - */

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;
  File_chooser file_chooser;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv,
    XV_USE_LOCALE, TRUE, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL,      argv[0],
    XV_WIDTH,         200,
    XV_HEIGHT,        100,
    0);

  file_chooser = xv_create(frame, FILE_CHOOSER,
    FILE_CHOOSER_TYPE,        FILE_CHOOSER_OPEN,
    FILE_CHOOSER_NOTIFY_FUNC, load_proc,
    0);

  panel = xv_create(frame, PANEL, 0);

  xv_create(panel, PANEL_BUTTON,
    PANEL_LABEL_STRING, "Datei laden",
    PANEL_NOTIFY_PROC,  choose_file_proc,
    PANEL_CLIENT_DATA,  file_chooser,
    0);

  xv_create(panel, PANEL_BUTTON,
    PANEL_LABEL_STRING, "Ende",
    PANEL_NOTIFY_PROC,  quit_proc,
    PANEL_CLIENT_DATA,  frame,
    0);

  xv_main_loop(frame);
  return(0);
 }
