#include <iostream>
#include <signal.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

bool user_interrupt = false;
void my_handler(int s){
           printf("Caught signal %d\n",s);
           user_interrupt=true;

}

int main(int argc, char** argv)
{
  int s;
  int nbytes;
  struct sockaddr_can addr;
  /*     struct can_frame { // defined in can.h
            canid_t can_id;  // 32 bit CAN_ID + EFF/RTR/ERR flags
            __u8    can_dlc; // frame payload length in byte (0 .. 8)
            __u8    __pad;   // padding
            __u8    __res0;  // reserved / padding
            __u8    __res1;  // reserved / padding
            __u8    data[8] __attribute__((aligned(8)));
    }; */
  struct can_frame frame;
  struct ifreq ifr;

  char const * ifname = "can0";

  if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while opening socket");
    return -1;
  }

  strcpy(ifr.ifr_name, ifname);
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

  if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("Error in socket bind");
    return -2;
  }

  frame.can_id  = 0x82;
  frame.can_dlc = 2;

  for (int i=0; i<8; i++)
  {
    frame.data[i] = i*16;
  }

 // frame.data[1] = 0x22;
  printf("size: %ld \n",sizeof(frame.can_id));
   while (!user_interrupt)
  {
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    nbytes = write(s, &frame, sizeof(frame));
    printf("Write %d bytes\n", nbytes);
    usleep(1000e3);

  }

  return EXIT_SUCCESS;
}
