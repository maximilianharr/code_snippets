#-*- coding: utf-8 -*-

class konto:

    def __init__(self,name,id,geld):
        self.name = name
        self.id = id
        self.geld = geld

    def einzahlen(z,geld):
        if (geld>0):
            self.geld = self.geld + geld

    def auszahlen(self,geld):
        if (geld<0):
            self.geld = self.geld - geld

    def info(self):
        print "Name : ",self.name
        print "Kontonummer : ",self.id
        print "Geldbetrag : ",self.geld

    def zahle_an(self,x,geld):
        if(geld>0)==(geld<self.geld):
            self.geld -= geld
            x.geld +=geld
            print "Geld wurde überwiesen"
            return True

a = konto("Max",1,100)
a.info()
b = konto("Mr.X",2,0)
b.info()

a.zahle_an(b,50)

a.info()
b.info()
