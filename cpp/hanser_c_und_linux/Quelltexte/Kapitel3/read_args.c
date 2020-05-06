/*
    read_args.c
*/

# include <stdio.h>
# include <unistd.h>

int main(int argc, char *argv[])
 {
  int option, i;
  char *out_filename=NULL;

  while ((option = getopt(argc, argv, "ho:")) >= 0)
    switch (option)
     {
      case 'h' : printf("Usage: %s [-o output-file] "
                        "[input-file ...]\n", argv[0]);
                 return(0);
      case 'o' : out_filename = optarg;
		 break;
      case '?' : return(1);     /* unbekannte Option */
     }

  for (i=optind; i<argc; i++)
    printf("'%s'\n", argv[i]);

  if (out_filename)
    printf("output is '%s'\n", out_filename);

  return(0);
 }
