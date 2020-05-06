/*
    modem.h - Funktionen zur Ansteuerung des Modems
*/

int open_modem(void);
void close_modem(void);
int reset_modem(void);
int dial(char *number);
