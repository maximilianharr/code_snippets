/*
    sprache3.c
*/

# include <stdio.h>
# include <locale.h>
# include <libintl.h>

int main()
 {
  bindtextdomain("sprache3", ".");
  textdomain("sprache3");
  setlocale(LC_MESSAGES, "");
  printf("%s\n", gettext("Hello world!"));
  printf("%s\n", gettext("This is a test."));

  return(0);
 }
