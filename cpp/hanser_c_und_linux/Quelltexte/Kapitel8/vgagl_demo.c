/*
    vgagl_demo.c - komplexere Grafik mit der LIBVGAGL
*/

# include <stdio.h>
# include <vga.h>
# include <vgagl.h>

int main()
 {
  int mode;
  GraphicsContext gc;

  if (vga_init())
    return(1);

  mode = G640x480x16;
  if (vga_setmode(mode))
    return(1);
  gl_setcontextvga(mode);
  gl_getcontext(&gc);	/* GraphicsContext holen */

  vga_setpalette(0, 58, 58, 58);
  vga_clear();

  gl_setcontextvgavirtual(mode);

  gl_setfont(8, 8, gl_font8x8);	/* Text vorbereiten */
  gl_setwritemode(FONT_COMPRESSED
    | WRITEMODE_OVERWRITE);
  gl_setfontcolors(15, 1);

  gl_write(10, 10, "Dies ist ein Test.");
  gl_printf(10, 20, "Breite=%d", vga_getxdim());
  gl_circle(320, 240, 100, 1);
  gl_fillbox(10, 400, 100, 70, 5);

  gl_copyscreen(&gc);	/* Grafik ins Video-RAM */

  vga_getch();		/* auf Taste warten */

  return(0);
 }
