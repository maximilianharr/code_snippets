# -*- coding: utf-8 -*-

import time
import random
import sys 

print('*********************************'); time.sleep(0.5)
print('*** SCHERE ** STEIN ** PAPIER ***'); time.sleep(0.5)
print('*********************************\n'); time.sleep(0.5)

print('Dabei ist die Schere die Nummer 1,'); time.sleep(0.5)
print('          der Stein  die Nummer 2,'); time.sleep(0.5)
print('          das Papier die Nummer 3,'); time.sleep(3.5)


regeln = { 2 : 1, 1 : 3, 3 : 2 }
print('Die Regeln sind simpel: Papier schlägt Stein, Stein schlägt Schere und Schere schlägt Papier')
time.sleep(0.0)

while 1:
  x = int(input("Wähle nun und gib die Zahl des Gegenstandes ein: "))

  random.seed()

  computer = random.randint(1,3)


  if x == computer: print('Unentschieden')
  elif regeln[x] == computer: print('Du hast gewonnen') #blinken
  else: print('Ich der Computer habe gewonnen')         #lange rot aufleuchten
  a = int(input("Noch eine Runde?  ;) JA=1, NEIN=0 "))  
  if a == 0: sys.exit()



