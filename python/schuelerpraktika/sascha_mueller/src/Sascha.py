# -*- coding: utf-8 -*-
import time
import random
import sys
#Kopf oder Zahl
print
print
print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'); time.sleep(0.5)
print('~~~~~~~~~~~~~~~~Kopf oder Zahl~~~~~~~~~~~~~~~~~~~~'); time.sleep(0.5)
print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n'); time.sleep(0.5)

print('Es gilt Kopf (Nummer 1)'); time.sleep(0.5)
print('        Zahl (nummer 2)'); time.sleep(0.5)

regeln = {1 : 2, 2 :1}
print('Die Regeln lauten wie Folgt, wenn Kopf und Kopf Übereinstimmen gewinnst du.\nDas selbe gilt auch für Zahl, wenn es aber verschieden sind, \nsprich Kopf und Zahl, Gewinnt der Computer.'); time.sleep(0.5)

while 1:
  x = int(input('Wähle nun Kopf oder Zahl mit der entsprechenden Zahl'))

  print('...'); time.sleep(0.75)
  print('...'); time.sleep(0.75)
  print('...'); time.sleep(0.75)
  print('...'); time.sleep(0.75)
  print('...'); time.sleep(0.75)

  random.seed()

  computer = random.randint(1,2)


  if x == computer: print('Du hast gewonnen!!!')
  elif regeln[x] == computer: print('Der Computer hat gewonnen!!!')

  a = int(input("Noch eine Runde?  ;) JA=1, NEIN=0"))  

  print('...'); time.sleep(1.0)
  print('...'); time.sleep(1.0)

  if a == 0: break

#Zahlen würfel
while 2:
  print
  print
  print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'); time.sleep(0.5)
  print('~~~~~~~~~~~~~~~~Zahlen würfeln~~~~~~~~~~~~~~~~~~~~'); time.sleep(0.5)
  print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'); time.sleep(0.5)

  print('Es muss eine höhere Zahl als der Computer gewürfelt werden. \n(Hier bei muss keine Taste betätigt werden!)'); time.sleep(3.0)
     
  while 1: 
    print('Und gewonnen hat ???'); time.sleep(1.5)
    print('...'); time.sleep(0.75)
    print('...'); time.sleep(0.75)
    print('...'); time.sleep(0.75)
    print('...'); time.sleep(0.75)
    print('...'); time.sleep(0.75)
    x = random.randint(1,6)
    y = random.randint(1,6)

    if x == y: print('Unentschieden'); time.sleep(0.75)
    elif x > y: print('Du hast gewonnen!!!'); time.sleep(0.75)
    else: print('Sry der Computer hat gewonnen!!!'); time.sleep(0.75)
    b = int(input('Noch eine Runde?  ;) JA=1, NEIN=0'))
    if b == 0: sys.exit()
