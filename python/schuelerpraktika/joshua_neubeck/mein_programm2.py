#-*- coding: utf-8 -*-

import random

print('Lotto')
print('Wähle 6 von 49 zahlen und du hast die Chance auf 5 mio Euro\n')

a = int(input("Wähle deine erste Zahl: "))
b = int(input("Wähle deine zweite Zahl: "))
c = int(input("Wähle deine dritte Zahl: "))
d = int(input("Wähle deine vierte Zahl: "))
e = int(input("Wähle deine fünfte Zahl: "))
f = int(input("Wähle deine sechste Zahl: "))

random.seed()

x = random.randint(1,49); print('Die richtigen zahlen lauten:', x) 

y = random.randint(1,49)
 while (x == y): # oder doch if?
 y = random.randint(1,49); print('Die richtigen zahlen lauten:', y)

z = random.randint(1,49)
 while (x == z) or (y == z):
 z = random.randint(1,49); print('Die richtigen zahlen lauten:', z)

w = random.randint(1,49) 
 while (x == w) or (y == w) or (z == w)
 w = random.radint(1,49); print('Die richtigen zahlen lauten:', w)

q = random.randint(1,49)
 while (x == q) or (y == q) or (z == q) or (w == q)
 q = random.radint(1,49); print('Die richtigen zahlen lauten:', q)

u = random.randint(1,49) 
 while (x == u) or (y == u) or (z == u) or (w == u) or (q == u)
 u = random.radint(1,49); print('Die richtigen zahlen lauten:', u)

#die lottozahlen der letzten ziejung lauten print('x,y,z...')




if a == x a==y a==z a == #die 3 36 möglichkeiten aaufzählen?
