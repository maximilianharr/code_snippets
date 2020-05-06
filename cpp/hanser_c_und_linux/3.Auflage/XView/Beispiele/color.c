/*
    color.c - Eine neue Farbe einrichten
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>
# include <xview/cms.h>

void quit_proc(Panel_item item, Event *event)
 {
  xv_destroy_safe(xv_get(item, PANEL_CLIENT_DATA));
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;
  Cms cms;	/* Color Map Segment */

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  cms = xv_create(0, CMS,
    CMS_SIZE,         CMS_CONTROL_COLORS+2,
    CMS_CONTROL_CMS,  TRUE,
    CMS_NAMED_COLORS, "red", "blue", NULL,
    0);
  if (cms == XV_NULL)
   {
    fprintf(stderr, "color: Can't create CMS.\n");
    return(1);
   }

  frame = xv_create(0, FRAME,
    FRAME_LABEL, "Farbdemo",
    XV_WIDTH,    300,
    XV_HEIGHT,   100,
    0);

  panel = xv_create(frame, PANEL,
    PANEL_BORDER, TRUE,
    WIN_CMS,      cms,
    0);

  xv_create(panel, PANEL_MESSAGE,
    PANEL_LABEL_STRING, "Jetzt auch in Farbe:",
    PANEL_ITEM_COLOR,   CMS_CONTROL_COLORS+0,
    PANEL_ITEM_Y,       20,
    0);

  xv_create(panel, PANEL_BUTTON,
    PANEL_LABEL_STRING, "Wirklich beenden?",
    PANEL_ITEM_COLOR,   CMS_CONTROL_COLORS+1,
    PANEL_NOTIFY_PROC,  quit_proc,
    PANEL_CLIENT_DATA,  frame,
    PANEL_ITEM_Y,       18,
    0);

  xv_main_loop(frame);
  return(0);
 }
