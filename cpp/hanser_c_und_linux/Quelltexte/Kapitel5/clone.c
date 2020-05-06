/*
    clone.c - Kind-Prozess mit clone() erzeugen
*/

# include <stdio.h>
# include <string.h>
# include <sched.h>
# include <signal.h>
# include <sys/wait.h>

static char buffer[80] = "Can you hear me?";

static char stack[10000];

int child_function(void *text)
 {
  printf("child is running.\n");
  strcat(buffer, text);
  return(0);
 }

int main()
 {
  int status;

  printf("buffer='%s'\n", buffer);

  printf("running child...\n");
  if (clone(&child_function, &(stack[10000]),
            CLONE_VM | SIGCHLD, " Yes!") == -1)
   {
    perror("clone: clone() failed");
    return(1);
   }

  wait(&status);
  printf("child returned %d.\n", status);
  printf("buffer='%s'\n", buffer);

  return(0);
 }
