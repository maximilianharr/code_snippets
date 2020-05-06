#-*- coding: utf-8 -*-
import time
import random
import sys
#Lotto
print
print
print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'); time.sleep(0.5)
print('~~~~~~~~~~~~~~~~~~~~~Lotto~~~~~~~~~~~~~~~~~~~~~~~~'); time.sleep(0.5)
print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n'); time.sleep(0.5)

print('Hallo dies ist die Lotto Maschine nenne sie 3 Ziffern \nim Zahlen Raum von 1 bis 5, wenn sie über ein stimmen \nhaben sie gewonnen!'); time.sleep(0.5)

regeln = {1 : 2, 2 : 1, 1 : 3, 3 : 1, 1 : 4, 4 : 1, 5 :1, 1 : 5, 2 : 3, 3 : 2, 4 : 2, 4 : 3, 2 : 4, 3 : 4, 2 : 5, 3 : 5, 4 : 5, 5 : 4, 5 : 3, 5 : 2}
while 1:
  s = 'Sie haben leider kein Glück!'

  while 1:
    x = int(input('Erste Ziffer eingeben:')); time.sleep(0.5)
    a = random.randint(1,5)

    print x ; time.sleep(0.5)
    print a ; time.sleep(0.5)
    if x == a: print('Sie haben die erste richtig geraten Glückwunsch!'); time.sleep(0.5)
    else: print(str(s)); time.sleep(0.5)
    break

  while 1:
    y = int(input('Zweite Ziffer eingeben:')); time.sleep(0.5)
    b = random.randint(1,5)

    print y ; time.sleep(0.5)
    print b ; time.sleep(0.5)
    if y == b: print('Sie haben die zweite richtig geraten Glückwunsch'); time.sleep(0.5)
    else: print(str(s)); time.sleep(0.5)
    break

  while 1:  
    z = int(input('Dritte Ziffer eingeben:')); time.sleep(0.5)
    c = random.randint(1,5)

    print z ; time.sleep(0.5)
    print c ; time.sleep(0.5)
    if z == c: print('Sie haben die dritte richtig geraten Glückwunsch'); time.sleep(0.5)
    else: print(str(s)); time.sleep(0.5)
    break

  a = int(input("Noch eine Runde?  ;) JA=1, NEIN=0")); time.sleep(0.5)
  if a == 0: sys.exit()
