/*
    new_child.c - einen Kind-Prozess erzeugen
*/

# include <stdio.h>
# include <unistd.h>

int main()
 {
  int child_pid, test;

  test = 1;
  printf("\t\ttest=%d, &test=%p\n", test, &test);

  if ((child_pid = fork()) == 0)
   {
    sleep(1);
    printf("Kind-Prozess:\ttest=%d, &test=%p\n",
           test, &test);
    test = 2;
    printf("Kind-Prozess:\ttest=%d, &test=%p\n",
           test, &test);
   }
  else
   {
    printf("Eltern-Prozess:\ttest=%d, &test=%p\n",
           test, &test);
    sleep(2);
    printf("Eltern-Prozess:\ttest=%d, &test=%p\n",
           test, &test);
   }

  return(0);
 }
