/*
    primzahl.c
*/

# include <stdio.h>

int ist_primzahl(int zahl)
 {
  int teiler=2;

  while (teiler*teiler <= zahl)
   {
    if (zahl % teiler == 0)
      return(0);       /* 'zahl' ist keine Primzahl */
    teiler++;
   }
  return(1);		/* 'zahl' ist eine Primzahl */
 }

int main()
 {
  int zahl;

  for (zahl=1; zahl<=100; zahl++)
    if (ist_primzahl(zahl))
      printf("%d\n", zahl);
  return(0);
 }
