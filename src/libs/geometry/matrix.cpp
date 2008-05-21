
/***************************************************************************
 *  matrix.cpp - A matrix class
 *
 *  Created: Wed Sep 26 13:54:12 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <geometry/matrix.h>
#include <geometry/vector.h>

#include <exception>
#include <cstdlib>
#include <cstdio>

namespace fawkes {

/** @class Matrix <geometry/matrix.h>
 * A general matrix class. It provides all the
 * operations that are commonly used with a matrix.
 * @author Daniel Beck
 */

/** Constructor.
 * @param num_rows number of rows
 * @param num_cols number of columns
 * @param data array containing elements of the matrix in row-order
 */
Matrix::Matrix( unsigned int num_rows,
		unsigned int num_cols,
		float* data )
{
  m_num_rows = num_rows;
  m_num_cols = num_cols;

  m_transposed = false;

  m_columns = (Vector**) malloc( m_num_cols * sizeof(Vector*) );

  for (unsigned int i = 0; i < m_num_cols; ++i)
    {
      if ( data )
	{ m_columns[i] = new Vector(m_num_rows, &data[i * m_num_rows]); }
      else
	{ m_columns[i] = new Vector(m_num_rows); }
    }
}


/** Copy-constructor.
 * @param m another matrix
 */
Matrix::Matrix(const Matrix& m)
{
  m_num_rows = m.m_num_rows;
  m_num_cols = m.m_num_cols;

  m_transposed = m.m_transposed;

  m_columns = (Vector**) malloc( m_num_cols * sizeof(Vector*) );

  for (unsigned int i = 0; i < m_num_cols; ++i)
    {
      m_columns[i] = new Vector(m_num_rows);
      (*m_columns[i]) = (*m.m_columns[i]);
    }
}


/** Destructor. */
Matrix::~Matrix()
{
  for (unsigned int i = 0; i < m_num_cols; ++i)
    {
      delete m_columns[i];
    }
  free(m_columns);
}


/** Determines the dimensions of the matrix.
 * @param num_cols pointer to an unsigned int to where the number of columns is copied to
 * @param num_rows pointer to an unsigned int to where the number of rows is copied to
 */
void
Matrix::size(unsigned int& num_rows,
	     unsigned int& num_cols) const
{
  num_rows = m_num_rows;
  num_cols = m_num_cols;
}


/** Sets the diagonal elements to 1.0 and all other to 0.0.
 * @return a reference to the matrix object
 */
Matrix&
Matrix::id()
{
  for (unsigned int row = 0; row < m_num_rows; row++)
  {
    for (unsigned int col = 0; col < m_num_cols; col++)
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
  unsigned int t;
  t = m_num_rows;
  m_num_rows = m_num_cols;
  m_num_cols = t;

  m_transposed = m_transposed ? false : true;

  return *this;
}


/** Computes a matrix that is the transposed of this matrix.
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
  if (m_num_rows == m_num_cols)
  {
    Matrix i(m_num_rows, m_num_rows);
    i.id();

    // for each column...
    for (unsigned int col = 0; col < m_num_cols; col++)
    {
      // ...multiply the row by the inverse of the element
      // on the diagonal...
      float factor = 1.0f / (*this)(col,col);
      i.mult_row(col, factor);
      this->mult_row(col, factor);

      // ...and substract that row multiplied by the elements
      // in the current column from all other rows.
      for (unsigned int row = 0; row < m_num_rows; row++)
      {
	if (row != col)
	{
	  float factor2 = (*this)(row,col);
	  i.sub_row(row, col, factor2);
	  this->sub_row(row, col, factor2);
	}
      }
    }

    *this = i;
  }
  else
    {
      printf("Matrix::invert(): Trying to compute inverse of non-quadratic matrix!\n");
      throw std::exception();
    }

  return *this;
}


/** Computes a matrix that is the inverse of this matrix.
 * @return a matrix that is the inverse of this matrix
 */
Matrix
Matrix::get_inverse() const
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
  if (m_num_rows != m_num_cols)
    {
      printf("Matrix::det(): The determinant can only be calculated for nxn matrices.\n");
      throw std::exception();
    }

  Matrix m(*this);
  float result = 1.0f;

  // compute the upper triangular matrix
  for (unsigned int col = 0; col < m_num_cols; col++)
    {
      float diag_elem = m(col, col);
      result *= diag_elem;

      // multiply n-th row by m(n,n)^{-1}
      m.mult_row( col, 1.0 / diag_elem );
      for (unsigned int row = col + 1; row < m_num_rows; row++)
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
Matrix::get_submatrix(unsigned int row,
		      unsigned int col,
		      unsigned int num_rows,
		      unsigned int num_cols) const
{
  if ( (row + num_rows) > m_num_rows )
    {
      num_rows = m_num_rows - row;
    }

  if ( (col + num_cols) > m_num_cols )
    {
      num_cols = m_num_cols - col;
    }

  Matrix m(num_rows, num_cols);

  for (unsigned int r = 0; r < num_rows; r++)
    {
      for (unsigned int c = 0; c < num_cols; c++)
	{
	  m(r,c) = (*this)(row + r, col + c);
	}
    }

  return m;
}


/** Overlays another matrix over this matrix.
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
  unsigned int max_row = (row + m.m_num_rows) > m_num_rows ? m_num_rows : (row + m.m_num_rows);
  unsigned int max_col = (col + m.m_num_cols) > m_num_cols ? m_num_cols : (col + m.m_num_cols);

  for (unsigned int r = row; r < max_row; r++)
    {
      for (unsigned int c= col; c < max_col; c++)
	{
	  (*this)(r,c) = m(r - row, c - col);
	}
    }
}


/** (Read-only) Access-operator.
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
  // TODO: sanity check

  float ret;
  if (m_transposed)
    { ret =  (*m_columns[row])[col]; }
  else
    { ret =  (*m_columns[col])[row]; }

  return ret;
}


/** (RW) Access operator.
 * see the read-only access operator for operational details
 * @param row the row of the element
 * @param col the column of the element
 * @return a reference to the specified element
 */
float&
Matrix::operator()(unsigned int row,
		   unsigned int col)
{
  // TODO: sanity check

  if (m_transposed)
    { return (*m_columns[row])[col]; }
  else
    { return (*m_columns[col])[row]; }
}


/** Assignment operator.
 * Copies the data form the rhs Matrix to the lhs Matrix.
 * @param m the rhs Matrix
 * @return a reference to this Matrix
 */
Matrix&
Matrix::operator=(const Matrix& m)
{
  for (unsigned int i = 0; i < m_num_cols; ++i)
    {
      delete m_columns[i];
    }
  free(m_columns);

  m.size(m_num_rows, m_num_cols);

  m_columns = (Vector**) malloc( m_num_cols * sizeof(Vector*) );

  for (unsigned int i = 0; i < m_num_cols; ++i)
    {
      m_columns[i] = new Vector(m_num_rows);
      (*m_columns[i]) = (*m.m_columns[i]);
    }

  return *this;
}


/** Matrix multiplication operator.
 * (Matrix)a.operator*((Matrix)b) computes a * b;
 * i.e., the 2nd matrix is right-multiplied to the 1st matrix
 * @param b the other matrix
 * @return the product of the two matrices (a * b)
 */
Matrix
Matrix::operator*(const Matrix& b) const
{
  const Matrix& a = (*this);

  if (a.m_num_cols != b.m_num_rows)
    {
      printf("Matrix::operator*(...): Dimension mismatch: a %d x %d matrix can't be multiplied "
	     "with a %d x %d matrix.\n",
	     a.m_num_rows, a.m_num_cols, b.m_num_rows, b.m_num_cols);
      throw std::exception();
    }

  unsigned int rows = a.m_num_rows;
  unsigned int cols = b.m_num_cols;

  Matrix result(rows, cols);

  for (unsigned int c = 0; c < cols; c++)
    {
      for (unsigned int r = 0; r < rows; r++)
	{
	  float t = 0.0f;

	  for (unsigned int i = 0; i < a.m_num_cols; i++)
	    {
	      t += a(r,i) * b(i,c);
	    }

	  result(r,c) = t;
	}
    }

  return result;
}


/** Combined matrix-multipliation and assignement operator.
 * @param m the rhs Matrix
 * @return a reference to the Matrix that contains the result of the multiplication
 */
Matrix&
Matrix::operator*=(const Matrix& m)
{
  *this = *this * m;

  return *this;
}


/** Multiply the matrix with given vector.
 * @param v a vector
 * @return the result of the matrix-vector multiplication
 */
Vector
Matrix::operator*(const Vector& v) const
{
  unsigned int max_cols = v.size();

  Vector result(m_num_rows);

  for (unsigned int r = 0; r < m_num_rows; ++r)
    {
      float row_result = 0.0;
      for (unsigned int c = 0; c < max_cols; ++c)
	{
	  row_result += (*this)(r, c) * v[c];
	}
      result[r] = row_result;
    }

  return result;
}


/** Mulitply every element of the matrix with the given scalar.
 * @param f a scalar
 * @return the result of the multiplication
 */
Matrix
Matrix::operator*(const float& f) const
{
  Matrix result(m_num_rows, m_num_cols);

  for (unsigned int i = 0; i < m_num_cols; ++i)
    {
      (*result.m_columns[i]) *= f;
    }

  return result;
}

/** Combined scalar multiplication and assignment operator.
 * @param f a scalar
 * @return reference to the result
 */
Matrix&
Matrix::operator*=(const float& f)
{
  for (unsigned int i = 0; i < m_num_cols; ++i)
    {
      (*m_columns[i]) *= f;
    }

  return *this;
}


/** Divide every element of the matrix with the given scalar.
 * @param f a scalar
 * @return the result of the divison
 */
Matrix
Matrix::operator/(const float& f) const
{
  Matrix result(m_num_rows, m_num_cols);

  for (unsigned int i = 0; i < m_num_cols; ++i)
    {
      (*result.m_columns[i]) /= f;
    }

  return result;
}


/** Combined scalar division and assignment operator.
 * @param f a scalar
 * @return reference to the result
 */
Matrix&
Matrix::operator/=(const float& f)
{
  for (unsigned int i = 0; i < m_num_cols; ++i)
    {
      (*m_columns[i]) /= f;
    }

  return *this;
}


/** Addition operator.
 * Adds the corresponding elements of the two matrices.
 * @param m the rhs matrix
 * @return the resulting matrix
 */
Matrix
Matrix::operator+(const Matrix& m) const
{
  if (m_num_rows != m.m_num_rows || m_num_cols != m.m_num_cols)
    {
      printf("Matrix::operator+(...): Dimension mismatch: a %d x %d matrix can't be added to a %d x %d matrix\n",
	     m_num_rows, m_num_cols, m.m_num_rows, m.m_num_cols);
      throw std::exception();
    }

  Matrix result(m_num_rows, m_num_cols);

  for (unsigned int row = 0; row < m_num_rows; row++)
    {
      for (unsigned int col = 0; col < m_num_cols; col++)
	{
	  result(row,col) = (*this)(row,col) + m(row,col);
	}
    }

  return result;
}


/**Add-assign operator.
 * @param m the rhs matrix
 * @return a reference to the resulting matrix (this)
 */
Matrix&
Matrix::operator+=(const Matrix& m)
{
  *this = *this + m;

  return *this;
}


/** Comparison operator.
 * @param m the rhs Matrix
 * @return true if every element of this matrix is equal to the
 * corresponding element of the other matrix
 */
bool
Matrix::operator==(const Matrix& m) const
{
  if (m_num_rows != m.m_num_rows || m_num_cols != m.m_num_cols)
    return false;

  for (unsigned int r = 0; r < m_num_rows; r++)
    {
      for (unsigned int c = 0; c < m_num_cols; c++)
	{
	  if ((*this)(r,c) != m(r,c))
	    return false;
	}
    }

  return true;
}


/** Changes the matrix by multiplying a raw with a factor.
 * @param row the row
 * @param the factor
 */
void
Matrix::mult_row(unsigned int row,
		 double factor)
{
  if (row > m_num_rows)
    {
      printf("Matrix::mult_row(...): Out of range: matrix has %d rows -- no %d-th row.\n",
	     m_num_rows, row);
      throw std::exception();
    }

  for (unsigned int col = 0; col < m_num_cols; col++)
    {
      (*this)(row,col) *= factor;
    }
}


/** For two rows A and B and a factor f, A is changed to A - f*B.
 * @param row_a the row that is changed
 * @param row_b the row that is substracted from row_a
 * @param factor the factor by which every element of row_b is multiplied before it is
 *        substracted from row_a
 */
void
Matrix::sub_row(unsigned int row_a,
		unsigned int row_b,
		float factor)
{
  if (row_a > m_num_rows || row_b > m_num_rows)
    {
      printf("Matrix::sub_row(...): Out of range: one of the argument \"row_a\"=%d or \"row_b\"=%d is greater "
	     "than the number of rows (%d)\n",
	     row_a, row_b, m_num_rows);
      throw std::exception();
    }

  for (unsigned int col = 0; col < m_num_cols; col++)
    {
      (*this)(row_a,col) -= factor * (*this)(row_b,col);
    }
}


/** Print matrix to standard out.
 * @param name a name that is printed before the content of the matrix
 */
void
Matrix::print_info(const char* name)  const
{
  if (name)
    { printf("%s:\n", name); }

  for (unsigned int r = 0; r < m_num_rows; ++r)
    {
      for (unsigned int c = 0; c < m_num_cols; ++c)
	{
	  printf("%f ", (*this)(r, c));
	}
      printf("\n");
    }
}

} // end namespace fawkes
