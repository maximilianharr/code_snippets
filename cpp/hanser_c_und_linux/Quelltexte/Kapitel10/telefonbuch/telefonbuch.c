/*
    telefonbuch.c - automatische Anwahl mit Modem
*/

# include <stdio.h>
# include <string.h>
# include <stdlib.h>
# include "telefonbuch.h"
# include "modem.h"
# include "eingabe.h"

char phonebook[MAX_ENTRIES][2][MAX_CHARS];

int get_entry(char *buffer, char *dest)
 {
  char c;
  int i;

  i = 0;
  while (((c = buffer[i]) != '\0')
	 && (c != '\t') && (c != '\n'))
   {
    if (i < MAX_CHARS-1)
      dest[i] = c;
    i++;
   }
  if (i < MAX_CHARS)
    dest[i] = '\0';
  else
    dest[MAX_CHARS-1] = '\0';
  return(i);
 }

int read_phonebook(char *name)
 {
  int i, k;
  char buffer[80];
  FILE *stream;

  strcpy(buffer, "sort ");
  strcat(buffer, name);
  if ((stream = popen(buffer, "r")) == NULL)
   {
    perror("telefonbuch");
    return(0);
   }

  i = 0;
  while (fgets(buffer, 80, stream) != NULL)
    if (strlen(buffer) > 1)
     {				/* Namen einlesen */
      k = get_entry(buffer, phonebook[i][0]);
      if (buffer[k] != '\t')
       {
        fprintf(stderr, "fehlerhafter Eintrag:"
	        " %s\n", buffer);
	pclose(stream);
        return(0);
       }
      while (buffer[++k] == '\t');
				/* Nummer einlesen */
      get_entry(&(buffer[k]), phonebook[i][1]);
      if (++i == MAX_ENTRIES)
       {
        fprintf(stderr, "Telefonbuch zu lang.\n");
	pclose(stream);
	return(i);
       }
     }

  pclose(stream);
  if (i == 0)
    fprintf(stderr, "kein Eintrag gefunden.\n");

  return(i);
 }


int main(int argc, char *argv[])
 {
  int n, error;

  if ((argc != 2) || (strcmp(argv[1], "-h") == 0))
   {
    printf("telefonbuch - automatische Anwahl mit"
           " einem analogen Modem.\n");
    printf("Aufruf: telefonbuch Datei\n");
    return(1);
   }

  if ((n = read_phonebook(argv[1])) < 1)
    return(1);

  if (error = open_modem())
   {
    fprintf(stderr, "telefonbuch: open_modem(): %s\n",
            strerror(error));
    exit(1);
   }

  if (error = reset_modem())
   {
    fprintf(stderr, "telefonbuch: reset_modem(): %s\n",
            strerror(error));
    exit(1);
   }

  error = eingabe(n);

  close_modem();
  return(error);
 }
