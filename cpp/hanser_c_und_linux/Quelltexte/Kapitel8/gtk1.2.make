#
# Universal-Makefile für GTK+2.0-Programme
#

default:
	@echo "Usage: make -f gtk1.2.make filename"
	@echo "Please use name of target without '.c'!"

%:	%.c
	gcc -Wall $< -o $@ `gtk-config --libs --cflags`
