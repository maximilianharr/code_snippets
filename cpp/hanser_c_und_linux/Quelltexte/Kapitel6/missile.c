/*
    missile.c - USB-Raketenwerfer ansteuern
*/

# include <stdio.h>
# include <string.h>
# include <unistd.h>
# include <usb.h>

# define ID_PRODUCT 0x0202	// Kennung des
# define ID_VENDOR 0x1130	//   Raketenwerfers

# define TIMEOUT 1000		// Millisekunden

char init1[8] = "USBC\0\0\4\0",
     init2[8] = "USBC\0\x40\2\0";

/*----- Funktion zum Auffinden des Gerätes -----*/

struct usb_device *find_missile(void)
 {
  struct usb_bus *bus;
  struct usb_device *udev;

  bus = usb_get_busses();  // Zeiger auf ersten USB

  while (bus != NULL)
   {
    udev = bus->devices;   // 1. Gerät
    while (udev != NULL)
     {
      if ((udev->descriptor.idVendor == ID_VENDOR)
          && (udev->descriptor.idProduct == ID_PRODUCT))
	return(udev);
      udev = udev->next;   // Zeiger auf nächstes Gerät
     }
    bus = bus->next;	   // Zeiger auf nächsten USB
   }
  return(NULL);
 }

/*----- Funktion zum Beanspruchen der Interfaces -----*/

int claim_missile(usb_dev_handle *hd)
 {
  usb_detach_kernel_driver_np(hd, 0); // ggf. Kernel-
  usb_detach_kernel_driver_np(hd, 1); // Treiber abkoppeln

  if (usb_set_configuration(hd, 1))
    return(1);
  if (usb_claim_interface(hd, 1))
    return(1);

  return(0);
 }

/*----- Funktion zum Senden von Kommandos -----*/

int missile_do(usb_dev_handle *hd, int left, int right,
               int up, int down, int fire)
 {
  int n;
  static char buffer[64];

  n = usb_control_msg(hd,
		      USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		      USB_REQ_SET_CONFIGURATION, 0, 1,
                      init1, sizeof(init1), TIMEOUT);
  if (n != sizeof(init1))
    return(1);

  n = usb_control_msg(hd,
                      USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		      USB_REQ_SET_CONFIGURATION, 0, 1,
                      init2, sizeof(init2), TIMEOUT);
  if (n != sizeof(init2))
    return(1);

  buffer[0] = 0;	// Kommandos in Puffer schreiben
  buffer[1] = left;
  buffer[2] = right;
  buffer[3] = up;
  buffer[4] = down;
  buffer[5] = fire;
  buffer[6] = buffer[7] = 8;
  for (n=8; n<64; n++)	// Rest mit Nullen auffüllen
    buffer[n] = 0;

  n = usb_control_msg(hd,
                      USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		      USB_REQ_SET_CONFIGURATION, 0, 1,
                      buffer, sizeof(buffer), TIMEOUT);
  if (n != sizeof(buffer))
    return(1);

  return(0);
 }

/*---------- Hauptprogramm ----------*/

int main()
 {
  struct usb_device *missile;
  usb_dev_handle *missile_hd;

  usb_init();		// libusb initialisieren

  usb_find_busses();	// alle USBs suchen
  usb_find_devices();	// alle Geräte an den USBs suchen

  if ((missile = find_missile()) == NULL)
   {
    fprintf(stderr, "missile: Device not found!\n");
    return(1);
   }

  missile_hd = usb_open(missile);   // Devide öffnen
  if (missile_hd == NULL)
   {
    perror("missile: Can't open device");
    return(1);
   }

  if (claim_missile(missile_hd))    // Interface belegen
   {
    perror("missile: Can't claim interface");
    usb_close(missile_hd);
    return(1);
   }
			//   <  >  ^  v  F
  if (missile_do(missile_hd, 0, 0, 1, 0, 1))   // hoch +
    perror("missile: Error sending message");  //   Feuer

  usleep(1500000L);

  if (missile_do(missile_hd, 0, 0, 0, 0, 0))   // stopp
    perror("missile: Error sending message");

  usleep(1500000L);

  if (missile_do(missile_hd, 0, 0, 0, 1, 0))   // runter
    perror("missile: Error sending message");

  usleep(1500000L);

  if (missile_do(missile_hd, 0, 0, 0, 0, 0))   // stopp
    perror("missile: Error sending message");

  usb_close(missile_hd);

  return(0);
 }
