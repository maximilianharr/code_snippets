/*
    vga_draw.c - SVGALIB-Malprogramm
*/

# include <stdio.h>
# include <vga.h>
# include <vgamouse.h>

int main()
 {
  int x, y, x_old, y_old, i, color;
  static unsigned char background[21][21];

  if (vga_getmousetype() == MOUSE_NONE)
   {
    fprintf(stderr, "No mouse configured.\n");
    return(1);
   }

  if (vga_init())
    return(1);

  vga_setmousesupport(1);	/* Maus vorbereiten */

  if (vga_setmode(G640x480x16))
    return(1);

  vga_setpalette(0, 24, 48, 60);	/* Hellblau */
  vga_setpalette(1, 0, 0, 0);		/* Schwarz */

  vga_clear();

  for (i=0; i<448; i++)		/* Palette zeichnen */
   {
    vga_setcolor(1+i/32);
    vga_drawline(0, i, 32, i);
   }
  vga_setcolor(1);	/* Box mit aktueller Farbe */
  vga_drawline(1, 478, 1, 449);
  vga_drawline(1, 478, 31, 478);
  vga_drawline(31, 478, 31, 449);
  vga_drawline(1, 449, 31, 449);
  color = 5;			/* aktuelle Farbe */
  vga_setcolor(color);
  for (i=451; i<477; i++)
    vga_drawline(3, i, 29, i);

  x = x_old = 320;		/* Startkoordinaten */
  y = y_old = 240;

  mouse_setposition(x, y);	/* Maus einstellen */
  mouse_setxrange(10, 629);
  mouse_setyrange(10, 469);
  mouse_setscale(32);

  for (i=0; i<21; i++)
    vga_getscansegment(background[i],
                       x-10, y-10+i, 21);
  vga_setcolor(1);
  vga_drawline(x-10, y, x+10, y);
  vga_drawline(x, y-10, x, y+10);

  do
   {
    mouse_waitforupdate();
    for (i=0; i<21; i++)
      vga_drawscansegment(background[i],
                          x-10, y-10+i, 21);
    x = mouse_getx();
    y = mouse_gety();
    if (mouse_getbutton() == MOUSE_LEFTBUTTON)
     {
      if (x > 32)		/* Zeichenflaeche */
       {
        vga_setcolor(color);
        vga_drawline(x_old, y_old, x, y);
	x_old = x;
	y_old = y;
       }
      else			/* Farbauswahl */
       {
        color = y/32+1;
	vga_setcolor(color);
	for (i=451; i<477; i++)
	  vga_drawline(3, i, 29, i);
       }
     }
    else
     {
      x_old = x;
      y_old = y;
     }
    for (i=0; i<21; i++)
      vga_getscansegment(background[i],
                         x-10, y-10+i, 21);
    vga_setcolor(1);
    vga_drawline(x-10, y, x+10, y);
    vga_drawline(x, y-10, x, y+10);
   }
  while (mouse_getbutton() != MOUSE_RIGHTBUTTON);

  return(0);
 }
