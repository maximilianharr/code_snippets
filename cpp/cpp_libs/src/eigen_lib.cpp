/**
 *  @file eigen_lib.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 08.01.2016
 *
 *  @brief Performs linear algebra calculations using Eigen library
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
#include <iostream>  /* Header that defines the standard input/output stream objects */
#include <cstdlib>  /* Header that defines several general purpose functions */
#include <string>
#include <eigen3/Eigen/Dense>  /* template library for linear algebra http://eigen.tuxfamily.org */
#include <math.h> /* Defines M_PI for pi value */
#include <eigen3/Eigen/Eigenvalues> 

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

/** @brief Standard command line parameter processing.
 *  @param Pass command line parameters
 *  @return 0 if -h flag is set
 */
int cmd_check(int, char*[]);

// GLOBAL VARIABLES


using namespace Eigen;
using namespace std;

/** @brief Function that returns eigen vector
  *  @param
  *  @return
  */
void eigen_fun(Eigen::MatrixXd& mat)
{
  mat = MatrixXd::Zero(2,2);
}

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  /* Check command line options. Stop execution if -h is set. */
  if(!cmd_check(argc,argv)) return 0;
  
  /* */
  MatrixXd mat(3,3); // MatrixSizeType (d=double, f=float, i=int, ...)
  Matrix2d mat2 = Matrix2d::Constant(0.1);
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
  cout << "Diagonal of mat2: \n" << mat.diagonal() << "\n";
  mat(2,2) = 10; /* Start counting at 0 */
  VectorXd vec(3);
  vec << 1,2,3;
  cout << "Here is mat*vec:\n" << mat*vec << endl;
  cout << "Here is vec . vec:\n" << vec.dot(vec) << endl;
  /* see also +,-,*,dot(),cross(),sum(),prod(),maxCoeff(),minCoeff(),mean(),trace(),diagonal(),
   * rows(),cols(),size(),transpose(),
   */
  cout << "This is mat:\n" << mat << "\n";
  cout << "Take a block from mat:\n" << mat.block(1,1,2,2) << "\n"; // (pos,pos,size,size)
  cout << "Take a block from vec:\n" << vec.head(2) << "\n";
  mat.block(1,1,2,2)=mat2.block(0,0,2,2);
  std::cout << "Insert mat2 in mat:\n" << mat << "\n";
  // Norm computations
  /* norm(),lpnorm(),lpNorm(),squaredNorm(),*/

  // Boolean reducations
  /* all(), any(), count() */

  // Solving (http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html)
  /* colPivHouseholder(),fullPivHouseholder(),eigenvectors(),eigenvalues(),AdjointEigenSolver,
   * determinant(),inverse(),solve(),SVD,QR,Cholesky,compute(),llt() ...
   * See also: http://eigen.tuxfamily.org/dox/group__TopicLinearAlgebraDecompositions.html
   */
  Matrix3f A;
  Vector3f b;
  A << 1,2,3,  4,5,6,  7,8,10;
  b << 3, 3, 4;
  cout << "Here is the matrix A:\n" << A << endl;
  cout << "Index A(0,1): " << A(0,1) << endl;
  cout << "Here is the vector b:\n" << b << endl;
  Vector3f x = A.fullPivHouseholderQr().solve(b);
  cout << "The solution is:\n" << x << endl;
  double relative_error = (A*x - b).norm() / b.norm(); // norm() is L2 norm
  cout << "The relative error is:\n" << relative_error << endl;
  // Furthermore (Sparse matrix manipulation, sparse linear systems,
  /* Jacobi Rotation, Householder transformation, */

  /* Nonlinear optimization (see /usr/include/eigen3/unsupported/Eigen)
   * (http://eigen.tuxfamily.org/dox/unsupported/group__NonLinearOptimization__Module.html)
   * eg. LevenbergMarquardt */

  /* TRANSFORMATIONS
   * Rotation2D,AngleAxis (3D Rotation) Quaternion,Scaling,Translation,Affine Transformation,
   * rotate(),tranlate(),
   */
  Matrix2f m;
  m = Rotation2Df(M_PI/2); /* roation in rad */
  std::cout << "2d roation with 90 degree\n" << m << "\n";
  /* Eigen Namespace: http://eigen.tuxfamily.org/dox/namespaceEigen.html */

  MatrixXd mat4;
  eigen_fun(mat4);

  MatrixXd iden;
  iden = MatrixXd::Identity(6,6);
  cout << "iden is: \n" << iden << "\n";

  cout << "This is A: \n" << A << "\n";
  cout << "This is A squared: \n" << A.cwiseProduct(A) << "\n";
  //VectorXcd eivals = A.eigenvalues();
  cout << "Eigenvalues of A: \n" << A.eigenvalues() << "\n";

  iden(5,5)=0;
  FullPivLU<MatrixXd> lu(iden);
  cout << "Rank of iden: " << lu.rank() << "\n";

  Eigen::MatrixXd mat5 = Eigen::MatrixXd::Identity(3,3);
  mat5(1,1) = 2;
  mat5(2,2) = 4;
  Eigen::MatrixXd mat_llt = Eigen::MatrixXd::Zero(3,3);

  std::cout << "mat5:\n" << mat5 << std::endl;
  mat_llt =  mat5.llt().matrixL(); 
  std::cout << "mat5.llt().matrixL():\n" << mat_llt << std::endl;
  mat_llt = mat_llt.inverse();
  std::cout << "mat5.inverse():\n" << mat_llt << std::endl;
  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////
int cmd_check(int argc, char* argv[])
{
  int option;
  /* third argument of getopt specifies valid options and whether they need input(:) */
  while((option = getopt(argc,argv,"hp:"))>=0)
  {
    switch (option)
    {
      case 'h': std::cout
                << "Usage: <filename> [options] \n\n"
                << "<desription> \n\n"
                << "Options: \n"
                << " -h                    show this help message and exit \n"
                << " -p <PARAMETER>        <description> \n"
                << " \n";
                return 0; /* do not execute main with this option */
      case 'p': std::cout << "-p = " << optarg << "\n"; /* optarg is option argument */
                break;
    }
  }
  return 1;
}


