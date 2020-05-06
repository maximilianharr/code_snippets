#
# Makefile für 'telefonbuch'
#

OBJ = telefonbuch.o modem.o eingabe.o
HDR = telefonbuch.h modem.h eingabe.h
LIB = -lncurses

telefonbuch:	$(OBJ)
		gcc $^ $(LIB) -o $@

%.o:	%.c $(HDR)
	gcc -c $<

clean:
	rm -f $(OBJ)
