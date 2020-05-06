/*
    pipe2.c - Pipes als Standardein- und -ausgabe
*/

# include <stdio.h>
# include <unistd.h>
# include <sys/wait.h>

int main()
 {
  int fd1[2], fd2[2], l;
  char buffer[80];

  if ((pipe(fd1) != 0) || (pipe(fd2) != 0))
   {
    perror("pipe2: pipe() failed");
    return(1);
   }

  if (fork() == 0)
   {
    close(fd1[1]);		    /* Kind-Prozess */
    close(fd2[0]);
    if ((dup2(fd1[0], STDIN_FILENO) == -1)
        || (dup2(fd2[1], STDOUT_FILENO) == -1))
     {
      perror("pipe2: dup2() failed");
      return(1);
     }
    close(fd1[0]);
    close(fd2[1]);
    execlp("sort", "sort", NULL);
    perror("pipe2: execlp() failed");
    return(1);
   }

  close(fd1[0]);		  /* Eltern-Prozess */
  close(fd2[1]);

  write(fd1[1], "These\nlines\nshall\nbe\nsorted\n", 28);
  close(fd1[1]);
  wait(NULL);

  if ((l = read(fd2[0], buffer, 79)) == -1)
    perror("pipe2: read() failed");
  else
    write(STDOUT_FILENO, buffer, l);

  close(fd2[0]);
  return(0);
 }
