#
# Makefile
#

primzahl:	primz_haupt.o primz_math.o
		gcc primz_haupt.o primz_math.o -lm \
		  -o primzahl

primz_haupt.o:	primz_haupt.c primz_math.h
		gcc -c primz_haupt.c

primz_math.o:	primz_haupt.c
		gcc -c primz_math.c

clean:
	rm -f primz_haupt.o primz_math.o
