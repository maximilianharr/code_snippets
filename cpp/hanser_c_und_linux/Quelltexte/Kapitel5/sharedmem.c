/*
    sharedmem.c
*/

# include <stdio.h>
# include <unistd.h>
# include <sys/shm.h>
# include <sys/ipc.h>
# include <sys/wait.h>

int main()
 {
  int shmem_id;
  char *buffer;

  if ((shmem_id = shmget(IPC_PRIVATE, 80, SHM_R | SHM_W))
      == -1)
   {
    perror("sharedmem: shmget() failed");
    return(1);
   }
  if ((buffer = shmat(shmem_id, 0, 0)) == (char *)-1)
   {
    perror("sharedmem: shmat() failed");
    shmctl(shmem_id, IPC_RMID, 0);
    return(1);
   }

  buffer[0] = '\0';	   /* Puffer initialisieren */

  if (fork() == 0)
   {				    /* Kind-Prozess */
    sprintf(buffer, "Message from child process");
    shmdt(buffer);
    return(0);
   }

  wait(NULL);			  /* Eltern-Prozess */
  printf("Child process wrote into buffer: '%s'\n",
         buffer);

  shmdt(buffer);
  shmctl(shmem_id, IPC_RMID, 0);
  return(0);
 }
