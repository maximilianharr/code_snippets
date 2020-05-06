/*
    usbcam.h - Kamera-Ansteuerung für WebCam
*/

int init_cam(char *device_name);

int get_image(int cam_fd, char *tmpfile, int swap_RGB);
