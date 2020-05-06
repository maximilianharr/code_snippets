/*
    find_usb.c - USB-Ger�te auflisten
*/

# include <stdio.h>
# include <string.h>
# include <usb.h>

int main()
 {
  int i;
  struct usb_bus *bus;
  struct usb_device *udev;
  usb_dev_handle *udevhd;
  static char buffer[256];

  usb_init();		// libusb initialisieren

  usb_find_busses();	// alle USBs suchen
  usb_find_devices();	// alle Ger�te an den USBs suchen

  bus = usb_get_busses();  // Zeiger auf ersten USB

  i = 1;
  while (bus != NULL)
   {
    printf("%d. Universal Serial Bus:\n", i++);
    udev = bus->devices;  // 1. Ger�t
    while (udev != NULL)
     {
      udevhd = usb_open(udev);
      if (udevhd != NULL)
       {
        if (udev->descriptor.iProduct)
	  usb_get_string_simple(udevhd,
	      udev->descriptor.iProduct, buffer, 256);
        else
	  buffer[0] = '\0';
	printf("  %04X : %04X  '%s'\n",
	       udev->descriptor.idVendor,
	       udev->descriptor.idProduct,
	       buffer);
        usb_close(udevhd);
       }
      udev = udev->next;  // Zeiger auf n�chstes Ger�t
     }
    bus = bus->next;	  // Zeiger auf n�chsten USB
   }

  if (i == 1)
    printf("No USB found.\n");

  return(0);
 }
