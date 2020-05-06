#
# Makefile für die XView-Beispielprogramme aus dem Buch
# "C und Linux", 3. Auflage, Hanser Verlag
# 2010 Martin Gräfe
#

INCPATH = -I/usr/openwin/include -I/usr/X11/include
LIBPATH = -L/usr/openwin/lib -L/usr/X11/lib
LIBS =    -lxview -lolgx -lX11

view_xpm:  LIBS = -lxview -lolgx -lX11 -lXpm

BINS = auswahl button-icon canvas2 canvas choose_file color frame \
       frame_cmd icon key_data menu notice2 notice panel quit2 quit \
       slider text view_ppm view_xpm xvdraw

all:	$(BINS)

icon:	icon.c console.icon console_mask.icon
	gcc -m32 $(INCPATH) $(LIBPATH) $@.c $(LIBS) -o $@

%:	%.c
	gcc -m32 $(INCPATH) $(LIBPATH) $@.c $(LIBS) -o $@

clean:	
	rm -f $(BINS)
