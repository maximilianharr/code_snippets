/**
 *  @file cplusplus_class.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 07.12.2015
 *
 *  @brief
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

// PROJECT INCLUDES

// LOCAL INCLUDES
#include "cplusplus_class2.hpp"

// FORWARD REFERENCES

//// METHOD DEFINITION /////////////////////////////////////////////////////////////////////////////

// LIFECYCLE
Square::Square(void) : side(1)
{

}

Square::Square(int a) : side(a)
{

}

Square::Square(const Square& from)
{

}

Square::~Square(void)
{

}

// OPERATORS

// OPERATIONS (all non-access/inquiry methods)

// ACCESS (attribute accessors)
int Square::area()
{
  return side*side;
}

// INQUIRY (Is*-methods)
