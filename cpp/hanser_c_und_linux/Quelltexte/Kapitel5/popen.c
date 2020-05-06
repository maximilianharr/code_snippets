/*
    popen.c - Unterverzeichnisse anzeigen
*/

# include <stdio.h>
# include <string.h>

int main()
 {
  int status;
  FILE *stream;
  char buffer[40];

  if ((stream = popen("ls -F", "r")) == NULL)
   {
    perror("popen: popen() failed");
    return(1);
   }

  while (fgets(buffer, 40, stream) != NULL)
    if (buffer[strlen(buffer)-2] == '/')
      printf("%s", buffer);

  status = pclose(stream);
  printf("(ls returned %d.)\n", status);
  return(0);
 }
