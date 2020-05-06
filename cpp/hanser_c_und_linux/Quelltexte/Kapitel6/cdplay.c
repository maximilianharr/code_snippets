/*
    cdplay.c - Audio-CD abspielen
*/

# include <stdio.h>
# include <unistd.h>
# include <stdlib.h>
# include <sys/ioctl.h>
# include <fcntl.h>
# include <linux/cdrom.h>

void err_exit(char *err_text, int return_code)
 {
  perror(err_text);
  exit(return_code);
 }

int main()
 {
  int fd;
  struct cdrom_tocentry toc_entry;
  struct cdrom_msf start_stop;

  if ((fd = open("/dev/cdrom", O_RDONLY)) == -1)
    err_exit("cdplay: Can't open /dev/cdrom", 1);

			/* Anfang des 1. Stuecks */
  toc_entry.cdte_track = 1;
  toc_entry.cdte_format = CDROM_MSF;
  if (ioctl(fd, CDROMREADTOCENTRY, &toc_entry) == -1)
    err_exit("cdplay: ioctl() failed", 1);
  start_stop.cdmsf_min0
    = toc_entry.cdte_addr.msf.minute;
  start_stop.cdmsf_sec0
    = toc_entry.cdte_addr.msf.second;
  start_stop.cdmsf_frame0
    = toc_entry.cdte_addr.msf.frame;

			/* Ende des letzten Stuecks */
  toc_entry.cdte_track = CDROM_LEADOUT;
  toc_entry.cdte_format = CDROM_MSF;
  if (ioctl(fd, CDROMREADTOCENTRY, &toc_entry) == -1)
    err_exit("cdplay: ioctl() failed", 1);
  start_stop.cdmsf_min1
    = toc_entry.cdte_addr.msf.minute;
  start_stop.cdmsf_sec1
    = toc_entry.cdte_addr.msf.second;
  start_stop.cdmsf_frame1
    = toc_entry.cdte_addr.msf.frame;

  if (ioctl(fd, CDROMPLAYMSF, &start_stop) == -1)
    err_exit("cdplay: ioctl() failed", 1);

  printf("Press <RETURN> to stop playing.\n");
  getchar();

  if (ioctl(fd, CDROMSTOP) == -1)
    err_exit("cdplay: ioctl() failed", 1);

  close(fd);
  return(0);
 }
