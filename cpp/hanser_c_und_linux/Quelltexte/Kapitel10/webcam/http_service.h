/*
    http_service.h - HTTP-Server f�r WebCam
*/

int http_service(int client_fd, int cam_fd, int delay,
                 int swapRGB, char *pass);
