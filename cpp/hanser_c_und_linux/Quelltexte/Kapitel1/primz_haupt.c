/*
    primz_haupt.c
*/

# include <stdio.h>
# include "primz_math.h"

int main()
 {
  int zahl;

  for (zahl=1; zahl<=100; zahl++)
    if (ist_primzahl(zahl))
      printf("%d\n", zahl);
  return(0);
 }
