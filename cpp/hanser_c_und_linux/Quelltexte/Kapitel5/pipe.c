/*
    pipe.c - Interprozesskommunikation mit einer Pipe
*/

# include <stdio.h>
# include <unistd.h>
# include <sys/wait.h>

int main()
 {
  int fd[2], l;
  char buffer[80];

  if (pipe(fd) != 0)
   {
    perror("pipe: pipe() failed");
    return(1);
   }

  if (fork() == 0)
   {
    close(fd[1]);		/* Kind-Prozess */
    if ((l = read(fd[0], buffer, 79)) == -1)
      perror("pipe: read() failed");
    else
     {
      buffer[l] = '\0';
      printf("Received string: '%s'\n", buffer);
     }
    close(fd[0]);
    return(0);
   }

  close(fd[0]);			/* Eltern-Prozess */
  sleep(1);
  write(fd[1], "Test!", 5);
  wait(NULL);

  close(fd[1]);
  return(0);
 }
