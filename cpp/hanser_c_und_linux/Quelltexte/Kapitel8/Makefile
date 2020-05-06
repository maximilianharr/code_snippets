#
# Universal-Makefile für GTK+2.0-Programme
#

default:
	@echo "Usage: make filename"
	@echo "Please use name of target without '.c'!"

%:	%.c
	gcc -Wall $< -o $@ \
	  `pkg-config --libs --cflags gtk+-2.0`
