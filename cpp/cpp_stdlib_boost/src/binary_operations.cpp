#include <iostream>
#include <signal.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


#include <bitset>

#define UBX_SYNC_CHAR_1 0xB5u    //!< First synchronization character of UBX Protocol
#define UBX_SYNC_CHAR_2 0x62u    //!< Second synchronization character of UBX Protocol
#define UBX_PREFIX      (UBX_SYNC_CHAR_2<<8|UBX_SYNC_CHAR_1) //!< UBX Protocol Prefix


unsigned long char_to_U4(char char_1, char char_2, char char_3, char char_4)
{
  std::bitset<8> byte_1,byte_2,byte_3,byte_4;
  std::bitset<32> result;
  unsigned long int_out = 0;

  byte_1 = char_1;
  byte_2 = char_2;
  byte_3 = char_3;
  byte_4 = char_4;

  for(int i=0; i<8; i++)
  { result[24+i] = byte_1[i]; }
  for(int i=0; i<8; i++)
  { result[16+i] = byte_2[i]; }
  for(int i=0; i<8; i++)
  { result[8+i] = byte_3[i]; }
  for(int i=0; i<8; i++)
  { result[0+i] = byte_4[i]; }

  int_out = result.to_ulong();
  return int_out;
}

int main(int argc, char** argv)
{

  char a = ' ';
  char b = '@';
  union ubx_byte
  {
    unsigned int i;
    char c;
  } ubx_val;

  unsigned int int_out = (0x62 << 8|0xB5);

  int a1 = (a<<8 | b);
  int a2 = (a<<8+b);
  std::bitset<16> b1(a1);
  std::bitset<16> b2(a2);

  std::cout << "a1: " << a1 << std::endl;
  std::cout << "b1: " << b1 << std::endl;
  std::cout << "a2: " << a2 << std::endl;
  std::cout << "b2: " << b2 << std::endl;

  char gnss_message[6];
  gnss_message[0] = 0xB5;
  gnss_message[1] = 0x62;
  std::cout << "0: " << gnss_message[0] << "1" << gnss_message[1] << std::endl;
  unsigned int value = char_to_U4('\0', '\0', gnss_message[1], gnss_message[0] );
  std::cout << value << std::endl;

  int a3 = 1026;
  int a4;
  a4 += a3 >> 8;
    std::cout << "a4: " << a4 << std::endl;
  a4 += char_to_U4('\0','\0','\0', a3);
  a4 += a3 >> 8;
  std::cout << "a4: " << a4 << std::endl;

  return EXIT_SUCCESS;
}
