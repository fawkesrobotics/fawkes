
/***************************************************************************
 *  matrix.cpp - A matrix class
 *
 *  Created: Wed Sep 26 13:54:12 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <utils/geometry/matrix.h>
#include <core/exception.h>

using namespace std;


/** @class Matrix libs/utils/geometry/matrix.h
 * A general matrix class. It provides all the 
 * operations that are commonly used with a matrix.
 */
 
/** @var Matrix::mNumRows
 * The number of rows the matrix has.
 */

/** @var Matrix::mNumCols
 * The number of columns the matrix has.
 */

/** @var Matrix::mData
 * A pointer to a field where the elements of the matrix are stored.
 */


/**Constructor.
 * @param num_rows number of rows
 * @param num_cols number of columns
 */
Matrix::Matrix( unsigned int num_rows,
		unsigned int num_cols )
{
  mNumRows = num_rows;
  mNumCols = num_cols;

  mData = new float[mNumRows * mNumCols];

  for (unsigned int i = 0; i < mNumRows * mNumCols; i++)
    {
      mData[i] = 0.0f;
    } 
}


/**Copy-constructor.
 * @param m another matrix
 */
Matrix::Matrix(const Matrix& m)
{
  m.get_dimensions(&mNumRows, &mNumCols);

  mData = new float[mNumRows * mNumCols];

  for (unsigned i = 0; i < mNumRows * mNumCols; i++)
    {
      mData[i] = m.mData[i];
    }
}


/**Destructor. */
Matrix::~Matrix()
{
  delete[] mData;

  mData = NULL;
}


/**Determines the dimensions of the matrix.
 * @param num_cols pointer to an unsigned int to where the number of columns is copied to
 * @param num_rows pointer to an unsigned int to where the number of rows is copied to
 */
void
Matrix::get_dimensions(unsigned int* num_rows,
		       unsigned int* num_cols) const
{
  *num_rows = mNumRows;
  *num_cols = mNumCols;
}


/** Sets the diagonal elements to 1.0 and all other to 0.0.
 * @return a reference to the matrix object
 */
Matrix&
Matrix::id()
{
  for (unsigned int row = 1; row <= mNumRows; row++)
  {
    for (unsigned int col = 1; col <= mNumCols; col++)
    {
      (*this)(row, col) = (row == col) ? 1.0 : 0.0;
    }
  }
  
  return *this;
}


/** Transposes the matrix.
 * A new field is allocated and the elements of the 
 * matrix are copied to the appropriate positions.
 * @return a reference to the matrix object now containing the transposed matrix
 */
Matrix&
Matrix::transpose()
{
  float* data = new float[mNumRows * mNumCols];
  
  for (unsigned int row = 0; row < mNumRows; row++)
  {
    for (unsigned int col = 0; col < mNumCols; col++)
    {
      unsigned int oldIndex = row * mNumCols + col;
      unsigned int newIndex = (mNumRows * (row * mNumCols + col)) % (mNumRows * mNumCols) + row;
      
      data[newIndex] = mData[oldIndex]; 
    }
  }
  
  delete[] mData;
  mData = data;
  
  unsigned int t;
  t = mNumRows;
  mNumRows = mNumCols;
  mNumCols = t;
  
  return *this;
}


/**Computes a matrix that is the transposed of this matrix.
 * 
 * @return a matrix that is the transposed of this matrix
 */
Matrix
Matrix::get_transpose() const
{
  Matrix m(*this);
  
  m.transpose();
  
  return m;
}


/** Inverts the matrix.
 * The algorithm that is implemented for computing the inverse 
 * of the matrix is the Gauss-Jordan-Algorithm. Hereby, the block-
 * matrix (A|I) consisting of the matrix to be inverted (A) and the
 * identity matrix (I) is transformed into (I|A^(-1)).
 * @return a reference to the matrix object which contains now the inverted matrix
 */
Matrix&
Matrix::invert()
{
  if (mNumRows == mNumCols)
  {
    Matrix i(mNumRows, mNumRows);
    i.id();
    
    // for each column...
    for (unsigned int col = 1; col <= mNumCols; col++)
    {
      // ...multiply the row by the inverse of the element
      // on the diagonal...
      float factor = 1.0f / (*this)(col,col);
      i.mult_row(col, factor);
      this->mult_row(col, factor);
      
      // ...and substract that row multiplied by the elements 
      // in the current column from all other rows.
      for (unsigned int row = 1; row <= mNumRows; row++)
      {
	if (row != col)
	{
	  float factor = (*this)(row,col);
	  i.sub_row(row, col, factor);
	  this->sub_row(row, col, factor);
	}
      }
    }
    
    *this = i;
  }
  else
    {
      cout << "Trying to compute inverse of non-quadratic matrix!" << endl;
      Exception e("Matrix::invert()");
      throw e;
    }

  return *this;
}


/**Computes a matrix that is the inverse of this matrix.
 * @return a matrix that is the inverse of this matrix
 */
Matrix
Matrix::get_invert() const
{
  Matrix m(*this);

  m.invert();

  return m;
}


/** Computes the determinant of the matrix.
 * @return the determinant
 */
float
Matrix::det() const
{
  if (mNumRows != mNumCols)
    {
      cout << "The determinant can only be calculated for nxn matrices." << endl;
      Exception e("Matrix::det()");
      throw e;
    }

  Matrix m(*this);
  float result = 1.0f;

  // compute the upper triangular matrix
  for (unsigned int col = 1; col <= mNumCols; col++)
    {
      float diag_elem = m(col, col);
      result *= diag_elem;

      // multiply n-th row by m(n,n)^{-1}
      m.mult_row( col, 1.0 / diag_elem );
      for (unsigned int row = col + 1; row <= mNumRows; row++)
	{
	  m.sub_row( row, col, m(row, col) );
	}
    }

  return result;
}


/** Returns a submatrix of the matrix.
 * @param row the row in the original matrix of the top-left element in the submatrix
 * @param col the column in the original matrix of the top-left element in the submatrix
 * @param num_rows the number of rows of the submatrix
 * @param num_cols the number of columns of the submatrix
 * @return the submatrix
 */
Matrix
Matrix::submatrix(unsigned int row,
		  unsigned int col,
		  unsigned int num_rows,
		  unsigned int num_cols) const
{
  if ( (row + num_rows - 1) > mNumRows )
    {
      num_rows = mNumRows - row + 1;
    }

  if ( (col + num_cols - 1) > mNumCols )
    {
      num_cols = mNumCols - col + 1;
    }

  Matrix m(num_rows, num_cols);

  for (unsigned int r = 1; r <= num_rows; r++)
    {
      for (unsigned int c = 1; c <= num_cols; c++)
	{
	  m(r,c) = (*this)(row + r - 1, col + c -1); 
	}
    }

  return m;
}


/**Overlays another matrix over this matrix.
 * @param row the top-most row from which onwards the the elements are
 * exchanged for corresponding elements in the given matrix
 * @param col the left-most column from which onwards the the elements
 * are exchanged for corresponding elements in the given matrix
 * @param m the other matrix
 */
void
Matrix::overlay(unsigned int row,
		unsigned int col,
		const Matrix& m)
{
  unsigned int max_row = (row + m.mNumRows - 1) >= mNumRows ? mNumRows : (row + m.mNumRows - 1);
  unsigned int max_col = (col + m.mNumCols - 1) >= mNumCols ? mNumCols : (col + m.mNumCols - 1);

  for (unsigned int r = row; r <= max_row; r++)
    {
      for (unsigned int c= col; c <= max_col; c++)
	{
	  (*this)(r,c) = m(r - row + 1, c - col + 1);
	}
    }
}


/**(Read-only) Access-operator.
 * With this operator it is possible to access a specific 
 * element of the matrix. To conform with the mathematical
 * fashion of specifying the elements of a matrix the top
 * left element of the matrix is accessed with (1, 1) 
 * (i.e., numeration starts with 1 and not with 0).
 * @param row the row of the element
 * @param col the column of the element
 * @return the value of the specified element
 */
float
Matrix::operator()(unsigned int row,
		   unsigned int col) const
{
  return mData[(row - 1) * mNumCols + col - 1];
}


/**(RW) Access operator.
 * see the read-only access operator for operational details
 * @param row the row of the element
 * @param col the column of the element
 * @return a reference to the specified element
 */
float&
Matrix::operator()(unsigned int row,
		   unsigned int col)
{
  return mData[(row - 1) * mNumCols + col - 1];
}


/**Assignment operator.
 * Copies the data form the rhs Matrix to the lhs Matrix.
 * @param m the rhs Matrix
 * @return a reference to this Matrix
 */
Matrix&
Matrix::operator=(const Matrix& m)
{
  m.get_dimensions(&mNumRows, &mNumCols);

  delete[] mData;

  mData = new float[mNumRows * mNumCols];

  for (unsigned int i = 0; i < mNumRows * mNumCols; i++)
    {
      mData[i] = m.mData[i];
    }

  return *this;
}


/**Matrix multiplication operator.
 * (Matrix)a.operator*((Matrix)b) computes a * b;
 * i.e., the 2nd matrix is right-multiplied to the 1st matrix
 * @param b the other matrix
 * @return the product of the two matrices (a * b)
 */
Matrix
Matrix::operator*(const Matrix& b) const
{
  const Matrix& a = (*this);

  if (a.mNumCols != b.mNumRows)
    {
      cout << "Dimension mismatch: a " << a.mNumRows << " x " << a.mNumCols 
	   << " matrix can't be multiplied with a " << b.mNumRows << " x " << b.mNumCols 
	   << " matrix." << endl;
      Exception e("Matrix::operator*(...)");
      throw e;
    }

  unsigned int rows = a.mNumRows;
  unsigned int cols = b.mNumCols;

  Matrix result(rows, cols);

  for (unsigned int c = 1; c <= cols; c++)
    {
      for (unsigned int r = 1; r <= rows; r++)
	{
	  float t = 0.0f;

	  for (unsigned int i = 1; i <= a.mNumCols; i++)
	    {
	      t += a(r,i) * b(i,c);
	    }

	  result(r,c) = t;
	}
    }

  return result;
}


/**Combined matrix-multipliation and assignement operator.
 * @param m the rhs Matrix
 * @return a reference to the Matrix that contains the result of the multiplication
 */
Matrix&
Matrix::operator*=(const Matrix& m)
{
  *this = *this * m;

  return *this;
}


/**Addition operator.
 * Adds the corresponding elements of the two matrices.
 * @param m the rhs matrix
 * @return the resulting matrix
 */
Matrix
Matrix::operator+(const Matrix& m) const
{
  if (mNumRows != m.mNumRows || mNumCols != m.mNumCols)
    {
      cout << "Dimension mismatch: a " << mNumRows << " x " << mNumCols 
	   << " matrix can't be added with a " << m.mNumRows << " x " << m.mNumCols 
	   << " matrix." << endl;
      Exception e("Matrix::operator+(...)");
      throw e;
    }

  Matrix result(mNumRows, mNumCols);

  for (unsigned int row = 1; row <= mNumRows; row++)
    {
      for (unsigned int col = 1; col <= mNumCols; col++)
	{
	  result(row,col) = (*this)(row,col) + m(row,col);
	}
    }

  return result;
}


/**Add-assigne operator.
 * @param m the rhs matrix
 * @return a reference to the resulting matrix (this)
 */
Matrix&
Matrix::operator+=(const Matrix& m)
{
  *this = *this + m;

  return *this;
}


/**Comparison operator.
 * @param m the rhs Matrix
 * @return true if every element of this matrix is equal to the
 * corresponding element of the other matrix
 */
bool
Matrix::operator==(const Matrix& m) const
{
  if (mNumRows != m.mNumRows || mNumCols != m.mNumCols)
    return false;

  for (unsigned int r = 1; r <= mNumRows; r++)
    {
      for (unsigned int c = 1; c <= mNumCols; c++)
	{
	  if ((*this)(r,c) != m(r,c))
	    return false;
	}
    }

  return true;
}


/**Changes the matrix by multiplying a raw with a factor.
 * @param row the row
 * @param the factor
 */
void
Matrix::mult_row(unsigned int row,
		 double factor)
{
  if (row > mNumRows)
    {
      cout << "Out of range: matrix has " << mNumRows << " rows -- no " 
	   << row << "-th row." << endl;
      Exception e("Matrix::mult_row(...)");
      throw e;
    }

  for (unsigned int col = 1; col <= mNumCols; col++)
    {
      (*this)(row,col) *= factor;
    }
}


/**For two rows A and B and a factor f, A is changed to A - f*B.
 * @param row_a the row that is changed
 * @param row_b the row that is substracted from row_a
 * @param factor the factor by which every element of row_b is multiplied before it is substracted from row_a
 */
void
Matrix::sub_row(unsigned int row_a,
		unsigned int row_b,
		float factor)
{
  if (row_a > mNumRows || row_b > mNumRows)
    {
      cout << "Out of range: one of the argument \"row_a\" = " << row_a
	   << " or \"row_b\" = " << row_b << " is greate than the number of rows ("
	   << mNumRows << ")." << endl;
      Exception e("Matrix::sub_row(...)");
      throw e;
    }
  
  for (unsigned int col = 1; col <= mNumCols; col++)
    {
      (*this)(row_a,col) -= factor * (*this)(row_b,col);
    }
}


/**Prints the matrix to a stream.
 * @param ostr the output stream
 */
void
Matrix::print_to_stream(std::ostream& ostr) const
{
  ostr << std::endl;
  for (unsigned int row = 1; row <= mNumRows; row++)
  {
    ostr << "(";
    for (unsigned int col = 1; col <= mNumCols; col++)
    {
      ostr << "[" << row << "," << col << "] "<< (*this)(row, col) << " ";
    }
    ostr << ")" << std::endl;
  }
}


/**This operator allows to output a matrix using the <<-operator.
 * @param ostr the output stream
 * @param m the matrix that is to be output
 * @return a reference to the output-stream
 */
std::ostream& operator<<(std::ostream &ostr, const Matrix& m)
{
  m.print_to_stream(ostr);
  return ostr;
}
