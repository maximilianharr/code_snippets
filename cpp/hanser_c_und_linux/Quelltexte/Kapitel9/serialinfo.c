/*
    serialinfo.c - Informationen ueber die
                   serielle Schnittstelle holen
*/

# include <stdio.h>
# include <unistd.h>
# include <string.h>
# include <fcntl.h>
# include <sys/ioctl.h>
# include <linux/serial.h>
# include <linux/serial_reg.h>
# if defined __GLIBC__
#   include <sys/io.h>
# else
#   include <asm/io.h>
# endif

int main(int argc, char *argv[])
 {
  int fd, base_adr, msr;
  struct serial_struct serial_port;

  if ((argc != 2) || (strcmp(argv[1], "-h") == 0))
   {
    printf("Usage: serialinfo device\n");
    return(0);
   }

  if ((fd = open(argv[1], O_RDWR)) == -1)
   {
    perror("serialinfo: Can't open device");
    return(1);
   }

  if (ioctl(fd, TIOCGSERIAL, &serial_port) == -1)
   {
    perror("serialinfo: ioctl() failed");
    return(1);
   }
  base_adr = serial_port.port;
  printf("Device:\t%s (COM%d)\nPort:\t0x%x\n"
         "IRQ:\t%d\n", argv[1], serial_port.line+1,
	 serial_port.port, serial_port.irq);

  ioperm(base_adr+UART_MSR, 1, 1);

  printf("Line status:\nCD  RI  DSR CTS\n");
  while(1)
   {
    msr = inb(base_adr+UART_MSR);
    printf("\r %d   %d   %d   %d",
           (msr & UART_MSR_DCD)? 1 : 0,
           (msr & UART_MSR_RI)?  1 : 0,
           (msr & UART_MSR_DSR)? 1 : 0,
           (msr & UART_MSR_CTS)? 1 : 0);
    fflush(stdout);
    usleep(100000L);
   }

  ioperm(base_adr+UART_MSR, 1, 0);
  close(fd);
  return(0);
 }
