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
#ifndef SQUARE_HPP /* Header File Guards. Prevent bad effect from double include. */
#define SQUARE_HPP /* Add Namespace if used. NAMSPACE_XX_H */

// SYSTEM INCLUDES

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

//// CLASS DEFINITION //////////////////////////////////////////////////////////////////////////////
class Square
{
  public:     /* Acessible from anywhere, where object is visible */
    friend class Rectangle;

// LIFECYCLE
  /** @brief Default constructor.
   *  @param
   *  @return
   */
  Square(void);
  /** @brief Default constructor.
   *  @param
   *  @return
   */
  Square(int);

  /** @brief Copy constructor.
   *  @param
   *  @return
   */
  Square(const Square& from);

  /** @brief Destructor.
   *  @param
   *  @return
   */
  ~Square(void);

// OPERATORS

// OPERATIONS (all non-access/inquiry methods)

// ACCESS (attribute accessors)
int area();

// INQUIRY (Is*-methods)

  protected:  /* Accessible from members and members of derived classes as well as friends */

  private:    /* Only accessible members or friends */
    int side;
};

#endif // SQUARE_HPP
