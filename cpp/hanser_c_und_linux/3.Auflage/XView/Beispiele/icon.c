/*
    icon.c - Fenster mit Programm-Icon
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/icon.h>

int main(int argc, char *argv[])
 {
  Frame frame;
  Icon frame_icon;
  Server_image icon_image, icon_mask_image;
  static unsigned short image_bits[256] = {
# include <images/console.icon>
    }, mask_image_bits[256] = {
# include <images/console_mask.icon>
    };

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL,  "Fenster mit Icon",
    FRAME_CLOSED, TRUE,
    0);

  icon_image = xv_create(0, SERVER_IMAGE,
    XV_WIDTH,          64,
    XV_HEIGHT,         64,
    SERVER_IMAGE_BITS, image_bits,
    0);

  icon_mask_image = xv_create(0, SERVER_IMAGE,
    XV_WIDTH,          64,
    XV_HEIGHT,         64,
    SERVER_IMAGE_BITS, mask_image_bits,
    0);

  frame_icon = xv_create(frame, ICON,
    ICON_IMAGE,      icon_image,
    ICON_MASK_IMAGE, icon_mask_image,
    ICON_WIDTH,      64,
    ICON_HEIGHT,     64,
    ICON_LABEL,      "mein Icon",
    0);

  xv_set(frame, FRAME_ICON, frame_icon, 0);

  xv_main_loop(frame);
  return(0);
 }
