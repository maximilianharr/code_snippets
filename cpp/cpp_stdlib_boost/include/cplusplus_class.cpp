/**
 *  @file cplusplus_class.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 30.11.2015
 *
 *  @brief	Description of class methods, constructors, ...
 *
 *
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *          https://google.github.io/styleguide/cppguide.html
 *
 *
 *  @bug
 *
 *
 *  @todo
 *
 *
 */

// PRAGMA

// SYSTEM INCLUDES
#include <string>
// PROJECT INCLUDES

// LOCAL INCLUDES
#include "cplusplus_class.hpp"
#include "cplusplus_class2.hpp"


// FORWARD REFERENCES

//// METHOD DEFINITION /////////////////////////////////////////////////////////////////////////////

int Rectangle::n = 0;

// LIFECYCLE
Rectangle::Rectangle(void)
  : width(0), height(0), ptr(nullptr)

{
  n++;
}

Rectangle::Rectangle(int a, int b)
  : width(a), height(b), ptr(nullptr)
{
  n++;
}

Rectangle::Rectangle(int a, int b, const std::string& str)
  : width(a), height(b), ptr(new std::string(str))
{
  n++;
}

Rectangle::~Rectangle(void)
{
  delete ptr;
  n--;
}

// OPERATORS
Rectangle Rectangle::operator+ (const Rectangle& param) /* const, since dont want to change param*/
{ Rectangle temp;
  temp.width = width + param.width;
  temp.height = height + param.height;
  return temp;
}

// OPERATIONS (all non-access/inquiry methods)
void Rectangle::convert(Square& a)
{
  width = a.side;
  height = a.side;
}

int Rectangle::area() const
{
  return width*height;
}
int Rectangle::area()
{
  return width*height;
}

// ACCESS (attribute accessors)
void Rectangle::setValue(int x, int y)
{
  width = x;
  height = y;
}

// INQUIRY (Is*-methods)
bool Rectangle::Isitme(Rectangle& param)
{
  if (&param == this) /* "this" is a pointer to the object whose member function is beeing exec. */
  {
    return true;
  }
  else
  {
    return false;
  }
}
