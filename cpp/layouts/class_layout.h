/**
 *  @file class_layout.h
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
#ifndef XX_HPP /* Header File Guards. Prevent bad effect from double include. */
#define XX_HPP /* Add Namespace if used. NAMSPACE_XX_H */

// SYSTEM INCLUDES

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

//// CLASS DEFINITION //////////////////////////////////////////////////////////////////////////////
class XX
{
  public:     /* Acessible from anywhere, where object is visible */

// LIFECYCLE
  /** @brief Default constructor.
   *  @param
   *  @return
   */
  XX(void);

  /** @brief Copy constructor.
   *  @param
   *  @return
   */
  XX(const XX& from);

  /** @brief Destructor.
   *  @param
   *  @return
   */
  ~XX(void);

// OPERATORS
  /** @brief Assignment operator.
   *  @param from the value to assign to this object.
   *  @return A reference to this object
   */
  XX operator= (const XX& from);

// OPERATIONS (all non-access/inquiry methods)
  /** @brief
   *  @param
   *  @return
   */
  void DoSth();

// ACCESS (attribute accessors)
  /** @brief
   *  @param
   *  @return
   */
  void set();

  /** @brief
   *  @param
   *  @return
   */
  void get();

// INQUIRY (Is*-methods)
  /** @brief
   *  @param
   *  @return
   */
  bool IsOpen();

  protected:  /* Accessible from members and members of derived classes as well as friends */

  private:    /* Only accessible members or friends */

};

#endif // XX_HPP
