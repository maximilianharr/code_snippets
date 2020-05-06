/*
    password.c - Passwortabfrage ohne Echo
*/

# include <stdio.h>
# include <unistd.h>
# include <string.h>
# include <fcntl.h>
# include <termios.h>

int main()
 {
  int old_flags;
  char password[16];
  struct termios term_attr;

  if (tcgetattr(STDIN_FILENO, &term_attr) != 0)
   {
    perror("password: tcgetattr() failed");
    return(1);
   }			 /* alte Einst. sichern */
  old_flags = term_attr.c_lflag;
  term_attr.c_lflag &= ~ECHO;
  if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &term_attr) != 0)
    perror("password: tcsetattr() failed");

  printf("password: ");
  scanf("%15s", password);
			/* Std.-Eingabe wie vorher */
  term_attr.c_lflag = old_flags;
  if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &term_attr) != 0)
    perror("password: tcsetattr() failed");

  if (strcmp(password, "secret") == 0)
    printf("\npassword accepted.\n");
  else
    printf("\nwrong password.\n");

  return(0);
 }
