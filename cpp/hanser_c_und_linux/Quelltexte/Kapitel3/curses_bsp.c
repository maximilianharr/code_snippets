/*
    curses_bsp.c
*/

# include <stdio.h>
# include <curses.h>

void print_pos(WINDOW *win)  /* Cursor-Position darst. */
 {
  int x, y;

  getyx(win, y, x);	/* aktuelle Pos. abfragen */
  attron(A_REVERSE);	/* inverse Darstellung ein */
  mvprintw(0, 0, "(%2d, %2d)", x, y);
  attroff(A_REVERSE);	/* inverse Darstellung aus */
  move(y, x);		/* Cursor wieder an alte Pos. */
  return;
 }

int main()			/* Hauptprogramm */
 {
  int	c, x, y;
  WINDOW *win;

  if ((win = initscr()) == NULL)
    return(1);
  cbreak();
  noecho();
  keypad(win, TRUE);	/* Sondertasten auswerten */

  move(1, 0);
  print_pos(win);

  while ((c = getch()) != KEY_END)  /* Ende = Abbruch */
   {
    switch(c)
     {
      case KEY_UP:    getyx(win, y, x); move(y-1, x);
	              break;
      case KEY_DOWN:  getyx(win, y, x); move(y+1, x);
	              break;
      case KEY_LEFT:  getyx(win, y, x); move(y, x-1);
	              break;
      case KEY_RIGHT: getyx(win, y, x); move(y, x+1);
	              break;
      case KEY_DC:    delch();
	              break;
      case KEY_BACKSPACE: getyx(win, y, x); move(y, x-1);
                      delch();
	              break;
      case KEY_IC:    insch(' ');
		      break;
      case KEY_HOME:  getyx(win, y, x); move(y, 0);
	              break;
      case KEY_F(1):  clear(); move(1, 0);
	              break;
      default:        if (c < KEY_MIN) addch(c);
     }
    print_pos(win);
   }

  endwin();
  return(0);
 }
