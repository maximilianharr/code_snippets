/*
    curses_bsp2.c - Farbdarstellung mit ncurses
*/

# include <stdio.h>
# include <curses.h>

int main()
 {
  int	 x, y;
  WINDOW *win;

  if ((win = initscr()) == NULL)
    return(1);
  start_color();  /* Farbausgabe initialisieren */
  cbreak();
  noecho();

  init_pair(1, COLOR_BLACK, COLOR_WHITE);
  init_pair(2, COLOR_YELLOW, COLOR_BLUE);
  init_pair(3, COLOR_RED, COLOR_CYAN);

  color_set(1, NULL);	/* Hintergrund weiﬂ */
  for (y=0; y<LINES; y++)
    for (x=0; x<COLS; x++)
      mvaddch(y, x, ' ');

  box(win, 0, 0);	/* Fenster einrahmen */

  mvprintw(2, 2, "Farbkombination 1");
  color_set(2, NULL);
  mvprintw(3, 2, "Farbkombination 2");
  color_set(3, NULL);
  mvprintw(4, 2, "Farbkombination 3");

  getch();	/* auf eine Taste warten */

  endwin();
  return(0);
 }
