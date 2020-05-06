#If,esle,while,for
# @todo for-Schleife
# @ask  wieso erscheint (ich bins) 5 mal

import random
print('hallo')

summe=0
for x in range(7):
  summe = summe+x
  print(summe)

for x in ('Hallo'):
  print (x, 'ich bins')
  
while 1:
  x = (int(input('!=1 oder ?=2')))
  a = random.randint(1,2)
  print x
  print a
  if x == a: print('!')
  else: print('?')
  break
