Vorgehensweise f�r Erzeugen von C-Projekten unter Windows.

Dev-Cpp installieren.
File > New > Project.
*.cpp mit "hello world" erzeugen. Compilieren (und testen).
Eine "Makefile.win" wird erzeugt (w�hlt Compiler etc.).
Makefile.win umbennen in beliebigen Namen (sozusagen als Template).
Project > Project Options > Makefile > Haken an "Use custom makefile (...)" und neue Makefile ausw�hlen.
Damit klappt das Compilieren. Anf�gen von Header, Libs etc. muss dann halt manuell erfolgen.
