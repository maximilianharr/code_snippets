/*
    ip-lookup.c - IP-Adresse einer Domain holen
*/

# include <stdio.h>
# include <string.h>
# include <arpa/inet.h>
# include <netdb.h>

int main(int argc, char *argv[])
 {
  struct hostent *host;
  struct in_addr *host_ip;

  if ((argc != 2) || (strcmp(argv[1], "-h") == 0))
   {
    fprintf(stderr, "Usage: ip-lookup domain-name\n");
    return(1);
   }

  host = gethostbyname(argv[1]);
  if (host == NULL)
   {
    herror("connect2: Can't get IP-address");
    return(1);
   }
  host_ip = (struct in_addr *) host->h_addr;

  printf("Hostname:\t%s\n", host->h_name);
  printf("IP-Address:\t%s\n", inet_ntoa(*host_ip));

  return(0);
 }
