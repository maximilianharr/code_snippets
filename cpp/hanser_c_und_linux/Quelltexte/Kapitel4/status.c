/*
    status.c - get status of file or directory.
*/

# include <stdio.h>
# include <unistd.h>
# include <sys/stat.h>
# include <time.h>
# include <string.h>

int main(int argc, char *argv[])
 {
  struct stat status;

  if ((argc != 2) || (strcmp(argv[1], "-h") == 0))
   {
    printf("Usage: status filename\n");
    return(1);
   }

  if (stat(argv[1], &status) != 0)
   {
    perror("status: Can't get file status");
    return(1);
   }

  printf("file/dir name:\t'%s'\n", argv[1]);
  printf("file/dir size:\t%ld bytes\n",
         status.st_size);
  if (S_ISDIR(status.st_mode))
    printf("type:\t\tdirectory\n");
  else if (S_ISREG(status.st_mode))
    printf("type:\t\tregular file\n");
  printf("protection:\t%o\n", status.st_mode & 0x1ff);
  printf("owner:\t\t%d\n", status.st_uid);
  printf("last modified:\t%s",
         ctime(&(status.st_mtime)));

  return(0);
 }
