/*
    eingabe.c - curses-Benutzerschnittstelle
*/

# include <string.h>
# include <unistd.h>
# include <ncurses.h>
# include <ctype.h>
# include "telefonbuch.h"
# include "modem.h"

		/*---- Hilfsfunktionen ----*/

void show_list(int n, int pos)
 {
  int i, start;

  start = (pos < LINES/2)? 0 : pos - LINES/2;
  erase();
  move(0, 0);
  attrset(A_NORMAL);
  for (i=start; i<pos; i++)
    printw("%-40s %s\n", phonebook[i][0],
           phonebook[i][1]);
  attrset(A_REVERSE);
  printw("%-40s %-38s\n", phonebook[pos][0],
         phonebook[pos][1]);
  i++;
  attrset(A_NORMAL);
  while ((i < n) && (i < LINES+start))
   {
    printw("%-40s %s\n", phonebook[i][0],
           phonebook[i][1]);
    i++;
   }
  refresh();
  return;
 }

int search(int n, char letter)
 {
  int i;

  letter = toupper(letter);
  for (i=0; i<n; i++)
    if (toupper(phonebook[i][0][0]) >= letter)
      return(i);
  return(n-1);
 }

void show_message(char *message)
 {
  int width;
  WINDOW *win;

  width = strlen(message)+4;
  win = newwin(5, width, LINES/2-6, (COLS-width)/2);
  box(win, 0, 0);
  mvwprintw(win, 2, 2, "%s", message);
  wrefresh(win);
  sleep(3);
  delwin(win);
  touchline(stdscr, LINES/2-6, 5);
  refresh();
  return;
 }

		/*---- Hauptfunktion ----*/

int eingabe(int n)
 {
  int c, pos;
  WINDOW *win;

  if ((win = initscr()) == NULL)
   {
    fprintf(stderr, "initscr() fehlgeschlagen.\n");
    return(1);
   }

  cbreak();
  noecho();
  curs_set(0);		/* Cursor unsichtbar */
  keypad(win, TRUE);	/* Sondertasten auswerten */

  pos = 0;
  show_list(n, pos);

  while ((c = getch()) != '\33')  /* ESC = Abbruch */
    if ((c >= 'A') && (c <= 'z'))
     {
      pos = search(n, c);
      show_list(n, pos);
     }
    else if (c == '\n')                /* Auswahl */
     {
      dial(phonebook[pos][1]);
      show_message("Bitte Hörer abnehmen!");
     }
    else if (c == KEY_UP)	/* Zeile hoch */
     {
      if (pos > 0)
	show_list(n, --pos);
     }
    else if (c == KEY_DOWN)	/* Zeile runter */
     {
      if (pos < n-1)
	show_list(n, ++pos);
     }
    else if (c == KEY_PPAGE)	/* Seite hoch */
     {
      pos -= LINES-1;
      if (pos < 0)
	pos = 0;
      show_list(n, pos);
     }
    else if (c == KEY_NPAGE)	/* Seite runter */
     {
      pos += LINES-1;
      if (pos >= n)
	pos = n-1;
      show_list(n, pos);
     }

  erase();
  refresh();
  endwin();
  return(0);
 }
