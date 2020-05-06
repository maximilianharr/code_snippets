/*
    slider.c - Schieberegler und Pegelanzeige
*/

# include <stdio.h>
# include <xview/frame.h>
# include <xview/panel.h>

void slider_proc(Panel_item item, int value,
                 Event *event)
 {
  Panel_gauge_item gauge;

  gauge = xv_get(item, PANEL_CLIENT_DATA);
  xv_set(gauge, PANEL_VALUE, value*value, 0);
  return;
 }

int main(int argc, char *argv[])
 {
  Frame frame;
  Panel panel;
  Panel_slider_item slider;
  Panel_gauge_item gauge;

  xv_init(XV_INIT_ARGC_PTR_ARGV, &argc, argv, 0);

  frame = xv_create(0, FRAME,
    FRAME_LABEL,       "Schieberegler",
    FRAME_SHOW_FOOTER, TRUE,
    0);

  panel = xv_create(frame, PANEL,
    PANEL_LAYOUT, PANEL_VERTICAL,
    0);

  slider = xv_create(panel, PANEL_SLIDER,
    PANEL_LABEL_STRING, "Wert:",
    PANEL_MIN_VALUE,    0,
    PANEL_MAX_VALUE,    20,
    PANEL_TICKS,        5,
    PANEL_SLIDER_WIDTH, 80,
    PANEL_NOTIFY_PROC,  slider_proc,
    0);

  gauge = xv_create(panel, PANEL_GAUGE,
    PANEL_LABEL_STRING, "Wert^2:",
    PANEL_MIN_VALUE,    0,
    PANEL_MAX_VALUE,    400,
    PANEL_TICKS,        5,
    PANEL_GAUGE_WIDTH,  120,
    0);

  xv_set(slider, PANEL_CLIENT_DATA, gauge, 0);

  window_fit(panel);
  window_fit(frame);
  xv_main_loop(frame);
  return(0);
 }
