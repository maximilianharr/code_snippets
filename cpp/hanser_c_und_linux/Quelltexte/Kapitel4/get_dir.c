/*
    get_dir.c
*/

# include <stdio.h>
# include <dirent.h>
# include <sys/types.h>
# include <string.h>

int main(int argc, char *argv[])
 {
  DIR *directory;
  struct dirent *dir_entry;

  if ((argc != 2) || (strcmp(argv[1], "-h") == 0))
   {
    printf("Usage: get_dir path\n");
    return(1);
   }

  if ((directory = opendir(argv[1])) == NULL)
   {
    perror("get_dir: Can't open directory");
    return(1);
   }

  while ((dir_entry = readdir(directory)) != NULL)
    printf("%s\n", dir_entry->d_name);

  closedir(directory);
  return(0);
 }
