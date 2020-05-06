#
# Makefile für WebCam
#

OBJ = webcam.o http_service.o usbcam.o
HDR = http_service.h usbcam.h
LIB = -ljpeg

webcam:	$(OBJ)
	gcc $^ $(LIB) -o $@

%.o:	%.c $(HDR)
	gcc -Wall -c $<

clean:
	rm -f $(OBJ)
