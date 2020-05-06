/*
    my_ls.c - "ls" in einem Kind-Prozess ausfuehren
*/

# include <stdio.h>
# include <unistd.h>
# include <sys/types.h>
# include <sys/wait.h>

int main(int argc, char *argv[])
 {
  pid_t pid;
  int status;

  if ((pid = fork()) == 0)
   {				/* Kind-Prozess */
    execlp("ls", "ls", "-F", argv[1], NULL);
    perror("my_ls: execlp() failed");
    return(1);
   }
  wait(&status);		/* Eltern-Prozess */
  printf("Child process exited with return code %d.\n",
         WEXITSTATUS(status));
  return(0);
 }
