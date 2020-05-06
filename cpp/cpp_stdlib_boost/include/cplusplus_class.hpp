/**
 *  @file class_layout.hpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 30.11.2015
 *
 *  @brief	Example/Template for C++ class.
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
#ifndef RECTANGLE_HPP /* Header File Guards. Prevent bad effect from double include. */
#define RECTANGLE_HPP /* Add Namespace if used. NAMSPACE_Rectangle_H */

// SYSTEM INCLUDES
#include <string>

// PROJECT INCLUDES

// LOCAL INCLUDES
#include "cplusplus_class2.hpp"

// FORWARD REFERENCES

//// CLASS DEFINITION //////////////////////////////////////////////////////////////////////////////
class Rectangle /* : public|protected|private parent_class */
    /* Specifies if the derived members of parent are as they are, protected or private.
     * What is not derived are: constructor/destructor, assignment operator, friends,
     * private members.*/
{

  public:     /* Acessible from anywhere, where object is visible */
    static int n; /* Not associated with the object of the class, class scope! */
    /* They are independent objects with static storage duration, only once defined in program */
    std::string * ptr;

// LIFECYCLE
  /** @brief Default constructor.
   *  @param
   *  @return
   */
  Rectangle();

  /** @brief Copy constructor.
   *  @param a specifies width
   *  @param b specifies height
   *  @return
   */
  Rectangle(int a, int b);
  Rectangle(int a, int b, const std::string &str);


  /** @brief Destructor.
   *  @param
   *  @return
   */
  ~Rectangle(void);

// OPERATORS
  /** @brief Assignment operator.
   *  @param from the value to assign to this object.
   *  @return A reference to this object
   */
  Rectangle operator+ (const Rectangle& param);

// OPERATIONS (all non-access/inquiry methods)
  /** @brief
   *  @param
   *  @return
   */
  void convert(Square&);

// ACCESS (attribute accessors)
  /** @brief Overloaded. Const member only called if object is constant
   *  @param
   *  @return heigt*width
   */
  int area() const;
  int area();

  /** @brief
   *  @param
   *  @return
   */
  friend Rectangle duplicate (const Rectangle&);
  //friend Rectangle duplicate(const Rectangle& param);

  /** @brief Set Rectangle dimensions.
   *  @param x is width
   *  @param y is height
   *  @return
   */
  void setValue(int x, int y);

// INQUIRY (Is*-methods)
  /** @brief
   *  @param
   *  @return
   */
bool Isitme(Rectangle& param);

  protected:  /* Accessible from members and members of derived classes as well as friends */

  private:    /* Only accessible members or friends */
  int width,height;
};

#endif // RECTANGLE_HPP
