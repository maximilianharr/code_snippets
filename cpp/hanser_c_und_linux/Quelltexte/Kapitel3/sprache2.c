/*
    sprache2.c
*/

# include <stdio.h>
# include <locale.h>
# include <libintl.h>

int main()
 {
  setlocale(LC_MESSAGES, "");
  printf("%s\n", dgettext("grep", "out of memory"));

  return(0);
 }
