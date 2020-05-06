/*
    number.c - Add line numbers to text file(s).
*/

# include <stdio.h>
# include <unistd.h>

int main(int argc, char *argv[])
 {
  int option, i, line;
  char *output_file=NULL;
  static char buffer[256];
  FILE *in_stream, *out_stream;

  while ((option = getopt(argc, argv, "ho:")) > 0)
    if (option == 'h')
     {
      printf("Usage: number [-o output-file] "
             "input-file ...\n");
      return(0);
     }
    else if (option == 'o')
      output_file = optarg;
    else
      return(1);

  if (optind == argc)
   {
    fprintf(stderr, "number: Missing file name. "
            "Type 'number -h' for help.\n");
    return(1);
   }

  if (output_file == NULL)
    out_stream = stdout;
  else if ((out_stream = fopen(output_file, "w")) == NULL)
   {
    perror("number: Can't open output file");
    return(1);
   }

  line = 1;
  for (i=optind; i<argc; i++)
   {
    if ((in_stream = fopen(argv[i], "r")) == NULL)
     {
      fprintf(stderr, "%s: Can't open file '%s' "
              "for input: ", argv[0], argv[i]);
      perror("");
      return(1);
     }
    while (fgets(buffer, 256, in_stream) != NULL)
      fprintf(out_stream, "%3d  %s", line++, buffer);
    fclose(in_stream);
   }
 
  if (output_file != NULL)
    fclose(out_stream);
  return(0);
 }
