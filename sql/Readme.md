# Install SQL:

## Install xampp from:
Download *.run file:  
https://www.apachefriends.org/index.html  
  
Make run file executeable:  
``` 
chmod +x xampp-linux-x64-7.2.7-0-installer.run 
```
  
Install:  
``` 
./xampp-linux-x64-7.2.7-0-installer.run 
```

## Configure passwort from http for databse:  
Copy config.inc.php to /opt/lampp/phpmyadmin  

## SQL quick reference sheet:
https://www.w3schools.com/sql/sql_update.asp  
https://dev.mysql.com/doc/refman/8.0/  

## Start:
Terminal:  
sudo /opt/lampp/lampp start  

Browser:  
http://localhost/phpmyadmin/  

## Connection to databas (Section 10, Lecture 90 of udemy SQL course)  
Follow instruction from Video. Note: I don't know if installation of xampp control panel is actually required, see end of section.

Create new folder in /opt/lampp/htdocs. eg. udemy-mysql-course  
open link in brower: [http://localhost/udemy-mysql-course/](http://localhost/udemy-mysql-course/)  
Make index.php editable for user:  
```
sudo chown kohaf1:kohaf1 /opt/lampp/htdocs/udemy-mysql-course/index.php
```
Create a user in phpmyadmin and add to index.php:
```
<?php
// 0OgVtPTBPu4QlrY4 << password
$pdo = new PDO (
	  "mysql:host=127.0.0.1:3306;dbname=udemy_mysql",
	  "udemy_mysql",
	  "0OgVtPTBPu4QlrY4"
);
?>
```

#### Install Xampp Control Panel  
https://askubuntu.com/questions/890818/ubuntu-16-04-how-to-start-xampp-control-panel  
``` 
sudo apt-get install gksu 
gksu gedit /usr/share/applications/xampp-control-panel.desktop 
```

Insert:  
```
[Desktop Entry]  
Encoding=UTF-8  
Name=XAMPP Control Panel  
Comment=Start and Stop XAMPP  
Exec=gksudo /opt/lampp/manager-linux-x64.run  
Icon=/opt/lampp/htdocs/favicon.ico  
Categories=Application  
Type=Application  
Terminal=false  
```

``` 
sudo apt-get update 
```
