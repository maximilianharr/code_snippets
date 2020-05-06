#-*- coding: utf-8 -*-
#Exceptions
# @todo catch implementieren
print
while 1:
  (x,y) = (5,0)
  try:
    z = x/y
  except ZeroDivisionError as e:
    z = e
  print z
  break
print
print
try:
   fh = open("Test.odt", "w")
   fh.write("This is my test file for exception handling")
except IOError:
   print "Error: can\'t find file or read data"
else:
   print "Written content in the file successfully"
   fh.close()
print
