#README
Example from [here](https://stackoverflow.com/questions/496664/c-dynamic-shared-library-on-linux)


---
## Building the shared library and linking against it:
```sh
g++ -fPIC -shared shared.cpp -o libshared.so
g++ main.cpp -o main.o -L. -lshared
```

---
## Changing the library without rebuilding the program  
1. Change shared.cpp
*  rebuild library using ```g++ -fPIC -shared shared.cpp -o libshared.so```
*  Execute > Done.

---
## Error: Program can not find shared libraries
./main: error while loading shared libraries: libshared.so: cannot open shared object file: No such file or directory

### Check if all shared libraries can be found
```sh
ldd main
```
> linux-vdso.so.1 =>  (0x00007ffd70b0c000)  
> libshared.so => not found  
> ...  

### Export current directory to LD_LIBRARY_PATH
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.  
env | grep 'LD_LIBRARY_PATH'  
```

### Check if all shared libraries can be found
```sh
ldd main
```
> linux-vdso.so.1 =>  (0x00007ffceb1d5000)  
> libshared.so => ./libshared.so (0x00007fe5c4398000)  
> ...  

