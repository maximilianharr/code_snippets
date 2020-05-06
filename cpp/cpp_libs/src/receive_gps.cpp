/**
 *  @file receive_gps.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 21.12.2015
 *
 *  @brief Included in ROS sensor_fusion_ros_tool in general_fun.h
 *          Example/Template for reading ublox EVK-M8N.
 * 				 	The receiver is plug-n-play. Read data: cat ttyACM0 | grep RMC
 *				 	Make sure you are in the right group to access devices.
 * 					Details in: "C und Linux", Kap 6.7, Martin Graefe
 *					See also http://libusb.sourceforge.net/api-1.0/api.html
 * 					http://castor.am.gdynia.pl/doc/libusb-0.1.12/html/index.html
 *
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *          https://google.github.io/styleguide/cppguide.html
 *
 *
 *  @bug
 *
 *
 *
 *  @todo make sure to compile with -lusb flag at the end.
 * 				g++ -std=c++14 -o <..> <..cpp> -lusb
 *
 *
 */

// PRAGMA
#define ID_PRODUCT_GPS 0x01A8 /* ID of EVK-M8N (see lsusb) */
#define ID_VENDOR_UBLOX 0x1546 /* ID of uBlux. cat /var/lib/usbutils/usb.its | grep 1546 */
#define ID_PRODUCT_PCANUSB 0x000C
#define ID_VENDOR_PEAK 0x0C72
#define TIMEOUT 1000

// SYSTEM INCLUDES
#include <iostream>

#include <usb.h>  /* sudo apt-get installlibusb-dev? */

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES
/** @brief Checks if USB device is connected and assigns pointer to device.
 *  @param id_vendor, USB vendor ID (see lsusb)
 *  @param id_product, USB product ID (see lsusb)
 *  @return nullptr if not connected
 */
int check_usb(int id_vendor,int id_prouct);

// GLOBAL VARIABLES

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  int udev_ublox;

  /* Search and assign uBlox GPS receiver. */
  udev_ublox = check_usb(ID_VENDOR_UBLOX,ID_PRODUCT_GPS);
  if (udev_ublox==1)
  { std::cout << "GPS-receiver not connected.\n"; }
  else
  { std::cout << "GPS-receiver connected.\n";}

  return EXIT_SUCCESS;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////
int check_usb(int id_vendor,int id_product)
{
  usb_init(); 					/* initialize libusb and use its functions */
  usb_find_busses(); 		/* search all USB ports */
  usb_find_devices(); 	/* search all USB devices */

  struct usb_bus * bus;
  struct usb_device * udev;

  bus = usb_get_busses(); 		/* Pointer to first element [libusb] */

  while(bus!=NULL) 						/* Search all busses */
  {
    udev = bus->devices;		 	/* First device */
    while(udev != NULL) 			/* Search all devices */
    {
      if(udev->descriptor.iProduct)
      {
        if (udev->descriptor.idVendor==id_vendor
            && udev->descriptor.idProduct==id_product)
        {
          return 0;
        }
      }
      udev = udev->next; /* Pointer to next element */
    }
    bus = bus->next; /* Pointer to next bus */
  }
  return 1;
}
