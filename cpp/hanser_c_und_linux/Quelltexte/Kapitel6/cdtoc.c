/*
    cdtoc.c - "Inhaltsverzeichnis" einer Audio-CD
*/

# include <stdio.h>
# include <unistd.h>
# include <errno.h>
# include <sys/ioctl.h>
# include <fcntl.h>
# include <linux/cdrom.h>

int main()
 {
  int fd, i;
  struct cdrom_tochdr toc_hdr;
  struct cdrom_tocentry toc_entry;

  if ((fd = open("/dev/cdrom", O_RDONLY)) == -1)
   {
    if (errno == ENOMEDIUM)
      fprintf(stderr, "cdtoc: No CD in drive.\n");
    else
      perror("cdtoc: Can't open /dev/cdrom");
    return(1);
   }

  if (ioctl(fd, CDROMREADTOCHDR, &toc_hdr) == -1)
   {
    perror("cdtoc: Can't get header");
    return(1);
   }
  printf("First track: %d, last track: %d\n",
         toc_hdr.cdth_trk0, toc_hdr.cdth_trk1);

  for (i=toc_hdr.cdth_trk0; i<=toc_hdr.cdth_trk1; i++)
   {
    toc_entry.cdte_track = i;
    toc_entry.cdte_format = CDROM_MSF;
    if (ioctl(fd, CDROMREADTOCENTRY, &toc_entry) == -1)
     {
      perror("cdtoc: Can't get table of contents");
      return(1);
     }
    printf(" %2d) %02d:%02d.%02d\n", i,
           toc_entry.cdte_addr.msf.minute,
           toc_entry.cdte_addr.msf.second,
	   (toc_entry.cdte_addr.msf.frame*100+37)/75);
   }

  close(fd);
  return(0);
 }
