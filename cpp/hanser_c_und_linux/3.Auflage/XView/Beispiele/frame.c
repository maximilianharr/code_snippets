/*
    frame.c - Ein einfaches XView-Fenster oeffnen
*/

# include <stdio.h>
# include <xview/frame.h>

int main(int argc, char *argv[])
 {
  Frame frame;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);
  frame = xv_create(0, FRAME, 0);
  xv_main_loop(frame);
  return(0);
 }
