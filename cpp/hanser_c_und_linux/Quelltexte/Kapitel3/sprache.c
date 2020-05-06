/*
    sprache.c
*/

# include <stdio.h>
# include <locale.h>

int main()
 {
  setlocale(LC_NUMERIC, "en_US");
  printf("PI=%.3f\n", 3.141593);
  setlocale(LC_NUMERIC, "de_DE");
  printf("PI=%.3f\n", 3.141593);

  return(0);
 }
