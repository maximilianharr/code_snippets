/*
    lines.c - Linien mit der SVGALIB darstellen
*/

# include <stdio.h>
# include <vga.h>

int main()
 {
  int i;

  if (vga_init())
    return(1);

  if (vga_setmode(G640x480x16))
    return(1);

  vga_clear();

  for (i=0; i<64; i++)
   {
    vga_setcolor(i % 16);
    vga_drawline(i*10, 0, 639-i*10, 479);
   }

  vga_getch();		/* auf Taste warten */

  return(0);
 }
