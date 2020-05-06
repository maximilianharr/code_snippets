/*
    LED-line.c
*/

# include <stdio.h>
# include <unistd.h>

# if defined __GLIBC__
#   include <sys/io.h>
# else
#   include <asm/io.h>
# endif

int get_lp_base(void)
 {
  int i, base_adr[3] = {0x3bc, 0x378, 0x278};

  for (i=0; i<3; i++)
   {
    outb_p(0, base_adr[i]);
    if (inb_p(base_adr[i]) == 0)
      return(base_adr[i]);
   }
  return(0);
 }

int main()
 {
  int i, j, port_adr;
  char light[12] = "        0 \r";

  if (iopl(3) != 0)
   {
    perror("LED-line: Can't set I/O permissions");
    return(1);
   }

  if ((port_adr = get_lp_base()) == 0)
   {
    fprintf(stderr,
            "LED-line: No parallel port found.\n");
    return(1);
   }
  printf("Parallel port found at 0x%x.\n", port_adr);

  i = 0;
  j = 1;
  while (1)
   {
    outb(1<<i, port_adr);
    write(1, &(light[i]), 11-i);
    usleep(100000L);	/* 100000 Mikrosek. warten */
    i += j;
    if ((i == 0) || (i == 7))
      j = -j;
   }

  return(0);
 }
