/**
 *  @file sendRawEth.c
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 04.01.2016
 *
 *  @brief Sends Ethernet frames
 *         Check TCP traffic: tcptrack -i  wlan0 (or eth0)
 *
 *         See: https://austinmarton.wordpress.com/2011/09/
 *
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *          https://google.github.io/styleguide/cppguide.html
 *
 *
 *  @bug
 *
 *
 *  @todo
 *
 *
 */

// PRAGMA

/* Set hardware MAC address of receiver (HWaddr of either eth0 or wlan0, depending on connection) */

#define MY_DEST_MAC0	0x3c //0xd4 //0x3c
#define MY_DEST_MAC1	0xa9 //0xc9 //0xa9
#define MY_DEST_MAC2	0xf4 //0xef //0xf4
#define MY_DEST_MAC3	0x2f //0xe8 //0x2f
#define MY_DEST_MAC4	0x66 //0xe0 //0x66
#define MY_DEST_MAC5	0xe8 //0xe8 //0xe8

/* Select whether frames are received via WLAN or ETHERNET */
#define DEFAULT_IF	"wlan0"
#define BUF_SIZ		1024

// SYSTEM INCLUDES
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/ether.h>
#include <unistd.h>

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

/** @brief
 *  @param
 *  @return
 */

// GLOBAL VARIABLES

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  /* Set Interface name */
  char ifName[IFNAMSIZ] = DEFAULT_IF;

  /* Open RAW socket to send on */
  int sockfd;
  if ((sockfd = socket(AF_PACKET, SOCK_RAW, IPPROTO_RAW)) == -1) {
      perror("Error: Could not open socket.\n");
  }

  /* Get the index of the interface to send on */
  struct ifreq if_idx;
  memset(&if_idx, 0, sizeof(struct ifreq));
  strncpy(if_idx.ifr_name, ifName, IFNAMSIZ-1);
  if (ioctl(sockfd, SIOCGIFINDEX, &if_idx) < 0)
      perror("SIOCGIFINDEX");

  /* Get the MAC address of the interface to send on */
  struct ifreq if_mac;
  memset(&if_mac, 0, sizeof(struct ifreq));
  strncpy(if_mac.ifr_name, ifName, IFNAMSIZ-1);
  if (ioctl(sockfd, SIOCGIFHWADDR, &if_mac) < 0)
      perror("SIOCGIFHWADDR");

  /* Construct the Ethernet header */
  char sendbuf[BUF_SIZ];
  struct iphdr *iph = (struct iphdr *) (sendbuf + sizeof(struct ether_header));
  struct sockaddr_ll socket_address;
  memset(sendbuf, 0, BUF_SIZ);

  /* Ethernet header */
  struct ether_header *eh = (struct ether_header *) sendbuf;
  eh->ether_shost[0] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[0];
  eh->ether_shost[1] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[1];
  eh->ether_shost[2] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[2];
  eh->ether_shost[3] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[3];
  eh->ether_shost[4] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[4];
  eh->ether_shost[5] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[5];
  eh->ether_dhost[0] = MY_DEST_MAC0;
  eh->ether_dhost[1] = MY_DEST_MAC1;
  eh->ether_dhost[2] = MY_DEST_MAC2;
  eh->ether_dhost[3] = MY_DEST_MAC3;
  eh->ether_dhost[4] = MY_DEST_MAC4;
  eh->ether_dhost[5] = MY_DEST_MAC5;

  /* Ethertype field */
  eh->ether_type = htons(ETH_P_IP);
  int tx_len = 0;
  tx_len += sizeof(struct ether_header);

  /* Set the packet data */
  sendbuf[tx_len++] = 0xdd;

  /* Index of the network device */
  socket_address.sll_ifindex = if_idx.ifr_ifindex;

  /* Address length*/
  socket_address.sll_halen = ETH_ALEN;

  /* Destination MAC */
  socket_address.sll_addr[0] = MY_DEST_MAC0;
  socket_address.sll_addr[1] = MY_DEST_MAC1;
  socket_address.sll_addr[2] = MY_DEST_MAC2;
  socket_address.sll_addr[3] = MY_DEST_MAC3;
  socket_address.sll_addr[4] = MY_DEST_MAC4;
  socket_address.sll_addr[5] = MY_DEST_MAC5;

  /* Send packet */
  while(1){
    if (sendto(sockfd, sendbuf, tx_len, 0, (struct sockaddr*)&socket_address, sizeof(struct sockaddr_ll)) < 0){
      printf("Send 1 failed\n");
    }
    else{
      printf("Sending data 1\n");
    }
    sleep(1);
  }

  return 0;
}

//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


