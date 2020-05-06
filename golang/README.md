## Installing go on Ubunutu

For compiling /running go programs:  
```$ sudo apt install golang-go```

For documentation (godoc, ...):  
```$ sudo apt install golang-golang-x-tools  ```


## Starting with GO
Very good hands-on tutorial: [Official GO-site](https://tour.golang.org/welcome/1)

Good intro: [here](https://www.youtube.com/watch?v=XCsL89YtqCs)  
```
mkdir ~/golang
mkdir ~/golang/src
mkdir ~/golang/src/helloworld
export GOPATH=$HOME/golang
export GOBIN=$HOME/golang/bin
```
Create source file and run using ```go run``` or install in bin-directory using ```go install```
