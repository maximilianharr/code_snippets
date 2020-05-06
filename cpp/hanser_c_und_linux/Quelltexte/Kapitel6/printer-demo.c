/*
    printer-demo.c - Druckerausgabe mit popen()
*/

# include <stdio.h>
# include <string.h>

int main()
 {
  int status;
  FILE *stream;

  if ((stream = popen("lpr", "w")) == NULL)
   {
    perror("printer-demo: popen() failed");
    return(1);
   }

  fprintf(stream, "%%!PS\n%%%%BoundingBox: 30 30 565 810\n"
	  "%%%%Orientation: Portrait\n%%%%EndProlog\n"
	  "100 700 moveto\n"
	  "/Times-Roman 24 selectfont\n"
	  "(Hallo Welt!) show\n"
	  "currentpoint pop 100 add 2 div\n"
	  "newpath 708 65 0 360 arc stroke\n"
	  "showpage\n");

  status = pclose(stream);

  printf("printder-demo: lpr returned %d.\n", status);

  return(0);
 }
