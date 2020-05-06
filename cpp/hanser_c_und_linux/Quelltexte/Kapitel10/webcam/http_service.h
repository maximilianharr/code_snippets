/*
    http_service.h - HTTP-Server für WebCam
*/

int http_service(int client_fd, int cam_fd, int delay,
                 int swapRGB, char *pass);
