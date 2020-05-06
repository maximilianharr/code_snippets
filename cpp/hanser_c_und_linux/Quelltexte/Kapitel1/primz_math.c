/*
    primz_math.c
*/

# include <math.h>

int ist_primzahl(int zahl)
 {
  int teiler=2;

  while (teiler <= sqrt(zahl))
   {
    if (zahl % teiler == 0)
      return(0);       /* 'zahl' ist keine Primzahl */
    teiler++;
   }
  return(1);		/* 'zahl' ist eine Primzahl */
 }
