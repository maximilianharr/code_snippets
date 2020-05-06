/*
    pthread.c - Kind-Prozess mit der libpthread
*/

# include <stdio.h>
# include <string.h>
# include <pthread.h>

static char buffer[80] = "Can you hear me?";

void *child_function(void *text)
 {
  static int status;

  printf("child is running.\n");
  strcat(buffer, text);
  status = 0;
  pthread_exit(&status);
 }

int main()
 {
  int *status_ptr;
  pthread_t child_thread;

  printf("buffer='%s'\n", buffer);

  printf("running child...\n");
  if (pthread_create(&child_thread, NULL,
                     &child_function, " Yes!"))
   {
    fprintf(stderr,
            "pthread: pthread_create() failed.\n");
    return(1);
   }

  if (pthread_join(child_thread, (void *)&status_ptr))
    fprintf(stderr, "pthread: pthread_join() failed.\n");
  else
    printf("child returned %d.\n", *status_ptr);

  printf("buffer='%s'\n", buffer);

  return(0);
 }
