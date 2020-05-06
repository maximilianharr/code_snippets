/* http://www.cplusplus.com/doc/tutorial */
/* Compile Program with "g++ --std=c++14 summary.cpp". This is a linker error, as gcc can not
 * determine if the language and its headers is c or c++*/

/* A directive whis is read and interpreted by preprocessor (before compiling). Replaces entire
 * content of the specified header or file. */
#include <cstdio> /* 1) angle-brackets used to include headers that compose the std-library */
#include <string> /* Whether the headers are files or exist in some other for is impl.def.*/
#include <cstring> /*definded macros: __LINE__, __FILE__, __DATE__, __TIME__, __cplusplus, ..*/
#include <cstdlib> /* in C known as <stdlib.h> */
#include <sstream>
#include <iostream> /* For cout*/
#include <fstream>
#include <typeinfo> /* used for typeid */
#include "cplusplus_class.hpp" /* 2) Quotes to include files. file searched in implementation-def
                                 * manner, generally includes current path. */
#include "cplusplus_class2.hpp" /* Caution, including cpp causes multiple definition error */


/* Alternative C++ array (container) that allows to be copied and does not decay into pointers */
#include <array>
/* New in C++ for dynamic memory.C uses malloc, calloc, realloc and free (can also be used in C++*/
#include <new>

/* Variables */
#define PI 3.14159

/* Namespace (to prevent name collision eg. between libraries) */
/* Group entities into different logical scopes. */
/* Using namespace only valid in defined block (like variables) */
//using namespace std;
namespace myName
{
    /* Template determined on instantiation (overloading of functions) */
    template <class myType, int N>
    void add(myType& c1, myType c2, const myType& c3) /* Ampersand "Pass by reference" */
    {   /* makes sense for large datatypes as copiying them is expensive */
        c1=N; /* alters input variable c1 */
        c2=N; /* c2 has been copied, does not effect input variable */
        /* c3=2; results in "error: assignment of read-only reference ‘c3’ " */
    }
}
inline void someSmallFunction(int c1, bool c2=0) {};
/* calling function causes overhead. Inline informs compiler to insert func code */
/* Most compilers already optimize code! */
/* c2 is optional. If not provided it is assigned 0 as value. */

/* This is a friend function of class Rectangle that can access its private members */
Rectangle duplicate(const Rectangle& param)
{
  Rectangle res;
  res.width = param.width*3;
  res.height = param.height*3;
  return res;
}

int main(int argc, char** argv)
{
/* BASICS OF C++ */
// INTEGERS
    /* normal initialization (dec,8bit) */
    short int a=2.7e1;
    /* constructor initialization(octal,16bit), new in C++ */
    long int b(033);
    /* uniform initialization(hex,32bit), new in C++11 [g++ ... -std=c++11] */
    /* The advantage of uniform init is that it is not confused with function declaration */
    long long int c{0x1B};
    /* Is constant [error: assignment of read-only variable ‘d’] */
    const float d{5.32};
    /* printf is C function but string is C++ structure */
    printf("%d %ld %lld %f \t %f \n",a,b,c,d,PI);

// STRINGS (Array of characters)
    /* ASCII or UTF-8 representation, 1 byte or 8 bit */
    char myChar1 {'a'};
    /* UTF-32 representation, new in C++11, wchar_t uses flexible format */
    char32_t myChar2 {'a'};
    char myChar3 [20] = {'M','a','x','\0'}; // \0 is end of string, or optionally foo[20] = "Max";
    /* Strings have dynamic size determined during runtime */
    std::string myString {"Hello world! \n"};
    /* c_str() returns constant char */
    printf("%s %s \n",myString.c_str(),myChar3);

// OPERATORS
    /* Assignment operator (=) */
    a=27;
    /* Arithmetric operator (+,-.*,/,%) */
    b=c/a*PI;
    /* Compound assignment (+=, -=, *=, /=, %=, >>=, <<=, &=, ^=, |=) */
    a+=1; /* or a=a+1; */
    /* Increment and decrement (++, --) */
    c=a++; /* c=a; a+=1; IF c=++a; THEN a+=1; c=a; */
    /* Relational and comparison operators ( ==, !=, >, <, >=, <= ) */
    bool e=(4>=PI);
    /* Logical operators (!, &&, ||) */
    bool f=!(1 && 0);
    /* Conditional ternary operator (?) */
    bool g=(4<=PI) ? 1:0; /* assigns 1 if true */
    /* Bitwise operators ( &, |, ^, ~, <<, >> ) */
    short int h1=3,h2=7,h;
    h=(h1<<1)&h2;/* 0011 wird zu 0110 und mit 1110 bleibt 0110 (6)*/
    /* Explicit type casting operator */
    a = (char) myChar1;
    /* Prefix (new, delete, sizeof
     * sizeof, returns size in bytes. Ints on 64bit stystem result in 8 */
    short int i = sizeof(myChar2);
    /* Scope qualifier or 'using namespace' */
    std::printf("%d %d %d %d %c %d \n",e,f,g,h,a,i);

/* PROGRAM STRUCTURE */

    /* if, else, break, continue, do while, switch case */
    short int n=0;
    do {
        printf("%d ",n);
        n++;
        if(n>=8) break;
    } while(n<=10);

    switch (n) {
    case 8: printf("Aborted\n"); break;
    case 10: printf("Success\n"); break;
    default: printf("Error\n");
    }

    /* references */
    int c1=1;
    int c2=1;
    myName::add<int,5>(c1,c2,1); /* second specifier cannot be a variable */
    printf(" c1 is now: %d \n c2 is now: %d \n",c1,c2);

    /* Arrays, multidimension must have bounds! Contiguous memory*/
    int myArray[][2]={{1,2},{3,4}};
    printf("%d \n",myArray[1][1]);
    std::array<int,2> myArray3 {3,4}; // Container defined in <array> header

    /* Pointers: Arrays can not be copied (for historical reasons) and thus a pointer is passed
     * Pointer needs data type that it points to. Size of pointer depends on platform.
     * (&) address of ... | (*) value pointed to by ... */
    double myNum = 1;
    double myNum2 = 2;
    double* sLoc1,* sLoc2; /* Caution int* a,b;results in b being an integer */
    const double* sLoc3 = &myNum; /* Read but not write access */
    double** sAll=&sLoc1; /* Pointer to pointer */
    /* *sLoc3=1; results in "error: assignment of read-only location ‘* sLoc3’ " */
    const void * const sLoc4 = &myNum; /* Constant read only pointer, flexible in data type! */
    /* Used eg in generic functions (generic = input data type is flexible) */
    printf("%p , %p , %f\n",sLoc1,sLoc2,*sLoc3);/*Dereference operator(*),NOT compound specifier!*/
    printf("Note that this is 48bit address space (12 hex*(2^4) numbers) as CPU manufactureres\n");
    printf("did not want to waste transistors if not used(256 TB is enough).Double is 8 byte.\n");
    double* sLoc5=nullptr; /* Pointer points to nowhere.Accessing its value results in Segm.Fault*/
    printf("%p \n",sLoc5);
    /* Pointer (ptrToFun) to function (someSmallFunction)! */
    void (*ptrToFun)(int,bool) = someSmallFunction;

    /* Dynamic memory [new+delete]. Usually memory needs to be determined before programm execution!
     * Memory alloc during runtime with "new" can be of variable size unlike arrays!
     * Since Computer memory is limited, system may deny allocation from Memory heap (exception) 
     * http://www.cplusplus.com/doc/tutorial/dynamic/
     * 
     * When to use new: [https://stackoverflow.com/questions/679571/when-to-use-new-and-when-not-to-in-c]
     * - when wish to keep object until explictly deleted
     * - new is more expensive than allocation in-place (use when needed)
     */
    int* sLoc6 = new int [5]; /* System dyn. allocates memory (from heap) and returns pointer */
    int* sLoc7 = new (std::nothrow) int [5]; /* bad_alloc not thrown, system continues if not catched */
    if (sLoc7 == nullptr) { printf("Allocation of sLoc7 denied. No exception thrown.");}
    delete sLoc6; /* Frees memory for next allocations with new */
    delete[] sLoc7; /* new arrays delete with [] */
    /* Note that C uses malloc, calloc, realloc and free for dynamic memory allocation */

    /* Data Structures. Can be nested. */
    struct GNSS_ellipsoid
    {
        double latitude;
        double longitude;
        double heigth;
    } GNSS_vehicle ;
    GNSS_vehicle.latitude=49.989898;
    GNSS_vehicle.longitude=8.40540;
    printf("%f, %f \n", GNSS_vehicle.latitude, GNSS_vehicle.longitude);
    /* Pointers to structures */
    GNSS_ellipsoid* GNSS_sign = &GNSS_vehicle; /* access of pointers with members with "->" */
    printf("%f \n", GNSS_sign->latitude); /* equivalent to (*GNSS_sign).latitude */

    /* typedef, alias for C++ types. Especially useful for abstract programs where it is unknown
     * wheather int or long has to be used. With typedef it can be replaced later on */
    typedef char C[2];
    C myLetter = {'h','i'}; /* "char[2] myLetter" does not work however */

    /* unions, allow memory portion to be accessed as different data type.
     * all members occupy same memory. */
    union hexCode /* size determined by biggest element */
    { /* eg used to access value in its entirety or as an array/struct of smaller elements */
        char c[5]; /* in UTF-8, see also char16_t, ... */
        long long int i;
    } hexRead;
    hexRead.i={0x6C65704F}; /* modification of one member effects all of them! */
    printf("hex-Code was %s \n",hexRead.c); /* Displays UTF-8 Code of char */

    /* enum types, set of custom identifiers. Derived object can take these enums as values */
    enum coordinateFrame {WGS84, IERS, vehicleFrame};
    coordinateFrame stopSign = WGS84;
    /* Can be converted in type int. Can also be specified. */
    if (stopSign == WGS84) {printf("%d \n",stopSign);}

/* COMPOUND DATA TYPES */

/* CLASSES */
  Rectangle rect;
  rect.setValue(4,5);
  Rectangle rect2{2,2}; /* uniform initialization, calling instructor. */
  printf("Area of rect: %d and %d \n",rect.area(), rect2.area());
  Rectangle * rect3;
  rect3 = new Rectangle(3,3);
  *rect3 = *rect3 + rect2; /* or rect3.operator+ (rect2) */
  printf("Area of rect3 after addition: %d and %d \n", rect3->area(), rect3->Isitme(rect2));
  printf("%d objects of class Rectangle have been created.\n", rect3->n);
  const Rectangle rect4 (1,2); /* Const obj. can  only call const members. Const. members cannot*/
  /* call non-const member nor modify non-static data. */
  printf("Area of arect4: %d \n", rect4.area());
  rect2 = *rect3; /* Calls default copy assignment */
  printf("Area of rect2 after copying is: %d \n", rect2.area());
  rect2 = duplicate(rect2);
  printf("Area of rect2 after duplication: %d \n ", rect2.area());
  delete rect3;
  Square sqr(4);
  rect2.convert(sqr);
  printf("Area of rect2 after converting: %d \n ", rect2.area());

/* OTHER LANGUAGE FEATUES */
  /* standard conversion */
  int Var1 = 2;
  double Var2;
  Var2=Var1;
  /* type casting dynamic_cast|reinterpret_cast|static_cast|const_cast */
  std::cout << "rect is of type: " << typeid(rect).name() << "\n";
  /* Exceptions. Provide a way to react to exceptional circumstances by transferring control to
   * special functions called handlers. Typically needed to check memory allocation. */
  try
  {
    int errortype=1;
    switch (errortype)
    {
    case 0: throw 20; /* Throws exceptions. Only accepts one parameter. */
      break;
    case 1: throw 'a';
      break;
    default: throw false;
      break;
    }
  }
  catch(int e) /* Handles exception, if data type equals the thrown data type. */
  {
    std::cout << "An exception occurred. Exception Nr. " << e << "\n";
  }
  catch(char e)
  {
    std::cout << "An exception occurred. Exception Nr. " << e << "\n";
  }
  /* C++ provides base class designed to declare objects to be thrown
   * std::exception (<exception> header)
   * eg. bad_alloc, bad_cast, bad_exception, bad_typeid, bad_function_call, runtime_error, ..*/

  /* Input and Output to files */
  std::ofstream myfile; /* cout is also of type ostream */
  myfile.open("example.txt",std::ios::in | std::ios::ate); /* also ios:: out,binary,ate,app,trunc*/
  myfile << "Hello World.\n";
  myfile.close(); /* Notifies OS that file is available again */
  /* see getline() for accessing text */
  /* State flags: bad(), fail(), eof(), good() *
   * Stream position [g=get,p=put]: tellg(), tellp(), seekg(), seekp(), */

  return EXIT_SUCCESS; /* Program ended successfully, else EXIT_FAILURE */


}
