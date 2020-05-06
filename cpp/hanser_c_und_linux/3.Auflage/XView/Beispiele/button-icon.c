/*
    button-icon.c - Schaltflaeche mit Grafik
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>

void quit_proc(Panel_item item, Event *event)
 {
  xv_destroy_safe(xv_get(item, PANEL_CLIENT_DATA));
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;
  Server_image button_image;
  static unsigned short image_bits[16]
    = {0x0180, 0x1998, 0x399c, 0x6186,
       0x6186, 0xc183, 0xc183, 0xc183,
       0xc183, 0xc003, 0xc003, 0x6006,
       0x6006, 0x381c, 0x1ff8, 0x07e0};

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  button_image = xv_create(0, SERVER_IMAGE,
    XV_WIDTH,          16,
    XV_HEIGHT,         16,
    SERVER_IMAGE_BITS, image_bits,
    0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL, "Bedienelement mit Grafik",
    0);

  panel = xv_create(frame, PANEL, 0);

  xv_create(panel, PANEL_MESSAGE,
    PANEL_ITEM_Y,       36,
    PANEL_LABEL_STRING, "Der Knopf zum Beenden:",
    0);

  xv_create(panel, PANEL_BUTTON,
    PANEL_ITEM_Y,      30,
    PANEL_LABEL_IMAGE, button_image,
    PANEL_LABEL_WIDTH, 50,
    PANEL_NOTIFY_PROC, quit_proc,
    PANEL_CLIENT_DATA, frame,
    0);

  window_fit(panel);
  window_fit(frame);
  xv_main_loop(frame);
  return(0);
 }
