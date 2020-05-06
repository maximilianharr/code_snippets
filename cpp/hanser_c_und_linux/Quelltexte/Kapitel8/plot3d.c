/*
    plot3d.c - 3D-Funktion zeichnen
*/

# include <stdio.h>
# include <vga.h>

# define f(x, y) (0.1/(0.1+(x)*(x)+(y)*(y)))

int main()
 {
  int i, j, xpos, ypos, xpos_old, ypos_old;
  double x, y, z;

  if (vga_init())
    return(1);
  if (vga_setmode(G640x480x16))
    return(1);

  vga_setpalette(0, 16, 44, 63);  /* hellblau */
  vga_setpalette(1, 0, 0, 0);     /* schwarz */

  vga_clear();

  vga_setcolor(1);
  for (i=0; i<=40; i++)	    /* horizontale Linien */
    for (j=0; j<=200; j++)
     {
      x = 0.01*j-1.0;
      y = 1.0-0.05*i;
      z = f(x, y)-2;
      xpos = 320+900*x/(y+4);
      ypos = -600*z/(y+4);
      if (j > 0)
        vga_drawline(xpos_old, ypos_old, xpos, ypos);
      xpos_old = xpos;
      ypos_old = ypos;
     }
  for (j=0; j<=40; j++)	    /* vertikale Linien */
    for (i=0; i<=200; i++)
     {
      x = 0.05*j-1.0;
      y = 1.0-0.01*i;
      z = f(x, y)-2;
      xpos = 320+900*x/(y+4);
      ypos = -600*z/(y+4);
      if (i > 0)
        vga_drawline(xpos_old, ypos_old, xpos, ypos);
      xpos_old = xpos;
      ypos_old = ypos;
     }

  vga_getch();		/* auf Taste warten */
  return(0);
 }
