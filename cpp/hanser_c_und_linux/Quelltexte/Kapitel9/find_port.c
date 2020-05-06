/*
    find_port.c
*/

# include <stdio.h>

# if defined __GLIBC__
#   include <sys/io.h>
# else
#   include <unistd.h>
#   include <asm/io.h>
# endif

int main()
 {
  int i, port = 0;
  const int base_adr[3] = {0x3bc, 0x378, 0x278};

  if (iopl(3) != 0)
   {
    perror("find_port: Can't set I/O permissions");
    return(1);
   }

  for (i=0; i<3; i++)
   {
    outb_p(0, base_adr[i]);
    if (inb_p(base_adr[i]) == 0)
      port = base_adr[i];
   }

  if (port == 0)
   {
    fprintf(stderr,
            "find_port: No parallel port found.\n");
    return(1);
   }
  printf("Parallel port found at 0x%x.\n", port);

  return(0);
 }
