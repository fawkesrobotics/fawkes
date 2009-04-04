
/***************************************************************************
 *  matrix.cpp - A matrix class
 *
 *  Created: Wed Sep 26 13:54:12 2007
 *  Copyright  2007-2009  Daniel Beck <beck@kbsg.rwth-aachen.de>
 *             2009       Masrur Doostdar <doostdar@kbsg.rwth-aachen.de>
 *             2009       Christof Rath <c.rath@student.tugraz.at>
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

#include "matrix.h"
#include "vector.h"

#include <core/exceptions/software.h>

#include <cstdlib>
#include <cstdio>
#include <algorithm>

#ifdef HAVE_OPENCV
#  include <opencv/cv.h>
#endif

namespace fawkes
{

/** @class Matrix <geometry/matrix.h>
 * A general matrix class.
 * It provides all the operations that are commonly used with a matrix, but has
 * been optimized with typical robotic applications in mind. That meas especially
 * that the chose data type is single-precision float and the class has been
 * optimized for small matrices (up to about 10x10).
 * @author Daniel Beck
 * @author Masrur Doostdar
 * @author Christof Rath
 */

/** @fn inline unsigned int Matrix::num_rows() const
 * Return the number of rows in the Matrix
 * @return the number of rows
 */

/** @fn inline unsigned int Matrix::num_cols() const
 * Return the number of columns in the Matrix
 * @return the number of columns
 */

/** @fn inline float* Matrix::get_data()
 * Returns the data pointer
 * @return the data pointer
 */

/** @fn inline const float* Matrix::get_data() const
 * Returns the const data pointer
 * @return the data pointer
 */

/** @fn float Matrix::data(unsigned int row, unsigned int col) const
 * (Read-only) Access to matrix data without index check.
 * With this operator it is possible to access a specific
 * element of the matrix.
 * Make sure the indices are correct, there is no sanity
 * check!
 * @param row the row of the element
 * @param col the column of the element
 * @return the value of the specified element
 */

 /** @fn float& Matrix::data(unsigned int row, unsigned int col)
 * (RW) Access  to matrix data without index check.
 * see the read-only access operator for operational details
 * Make sure the indizes are correct, there is no sanity
 * check!
 * @param row the row of the element
 * @param col the column of the element
 * @return a reference to the specified element
 */

/** Constructor.
 * @param num_rows number of rows
 * @param num_cols number of columns
 * @param data array containing elements of the matrix in row-by-row-order
 * @param manage_own_memory if true, the Matrix will manage its memory on its own, else it
 *        will not allocate new memory but works with the provided array
 */
Matrix::Matrix(unsigned int num_rows, unsigned int num_cols,
               float *data, bool manage_own_memory)
{
  m_num_rows = num_rows;
  m_num_cols = num_cols;
  m_num_elements = m_num_rows * m_num_cols;

  if (!m_num_elements) printf("WTF?\n");

  if (data == NULL || manage_own_memory)
  {
    m_data = (float*) malloc(sizeof(float) * m_num_elements);
    m_own_memory = true;

    /* It showed that for arrays up to approx. 1000 elements an optimized for-loop
     * is faster than a memcpy() call. It is assumed that the same is true for
     * memset(). */
    if (data != NULL)
    {
      for (unsigned int i = 0; i < m_num_elements; ++i) m_data[i] = data[i];
    }
    else
    {
      for (unsigned int i = 0; i < m_num_elements; ++i) m_data[i] = 0.f;
    }
  }
  else
  {
    m_data = data;
    m_own_memory = false;
  }
}

/** Copy-constructor.
 * @param tbc matrix to be copied
 */
Matrix::Matrix(const Matrix &tbc)
{
  m_num_rows   = tbc.m_num_rows;
  m_num_cols   = tbc.m_num_cols;
  m_num_elements = tbc.m_num_elements;

  m_own_memory = true;

  m_data = (float*) malloc(sizeof(float) * m_num_elements);
  for (unsigned int i = 0; i < m_num_elements; ++i) m_data[i] = tbc.m_data[i];
}

/** Destructor. */
Matrix::~Matrix()
{
  if (m_own_memory) free(m_data);
}

/** Determines the dimensions of the matrix.
 * @param num_cols pointer to an unsigned int to where the number of columns is copied to
 * @param num_rows pointer to an unsigned int to where the number of rows is copied to
 */
void
Matrix::size(unsigned int &num_rows, unsigned int &num_cols) const
{
  num_rows = this->num_rows();
  num_cols = this->num_cols();
}


/** Sets the diagonal elements to 1.0 and all other to 0.0.
 * @return a reference to the matrix object
 */
Matrix &
Matrix::id()
{
  for (unsigned int row = 0; row < num_rows(); ++row)
  {
    for (unsigned int col = 0; col < num_cols(); ++col)
    {
      data(row, col) = (row == col) ? 1.0 : 0.0;
    }
  }

  return *this;
}

/** Creates a quadratic matrix with dimension size and sets the diagonal elements to 1.0.
 * All other elements are set to 0.0.
 * @param size dimension of the matrix
 * @param data_buffer if != NULL the given float array will be used as data internal data store
 *        (the object will not perform any memory management in this case)
 * @return the id matrix object
 */
Matrix
Matrix::get_id(unsigned int size, float *data_buffer)
{
  return get_diag(size, 1.f, data_buffer);
}

/** Creates a quadratic matrix with dimension size and sets the diagonal elements to value.
 * All other elements are set to 0.0.
 * @param size dimension of the matrix
 * @param value of the elements of the main diagonal
 * @param data_buffer if != NULL the given float array will be used as data internal data store
 *        (the object will not perform any memory management in this case)
 * @return the diag matrix object
 */
Matrix
Matrix::get_diag(unsigned int size, float value, float *data_buffer)
{
  Matrix res(size, size, data_buffer, data_buffer == NULL);

  if (data_buffer != NULL)
  {
    unsigned int diag_elem = 0;
    for (unsigned int i = 0; i < size * size; ++i)
    {
      if (i == diag_elem)
      {
        diag_elem += size + 1;
        data_buffer[i] = value;
      }
      else data_buffer[i] = 0.f;
    }
  }
  else for (unsigned int i = 0; i < size; ++i) res.data(i, i) = value;

  return res;
}

/** Transposes the matrix.
 * @return a reference to the matrix object now containing the transposed matrix
 */
Matrix &
Matrix::transpose()
{
#ifdef HAVE_OPENCV
  if (m_num_cols == m_num_rows)
  {
    CvMat cvmat = cvMat(m_num_rows, m_num_cols, CV_32FC1, m_data);
    cvTranspose(&cvmat, &cvmat);

    return *this;
  }
#endif
  if (m_num_cols == m_num_rows) // Perform a in-place transpose
  {
    for (unsigned int row = 0; row < m_num_rows - 1; ++row)
    {
      for (unsigned int col = row + 1; col < m_num_cols; ++col)
      {
        float &a = data(row, col);
        float &b = data(col, row);
        float t = a;
        a = b;
        b = t;
      }
    }
  }
  else // Could not find a in-place transpose, so we use a temporary data array
  {
    float *new_data = (float*) malloc(sizeof(float) * m_num_elements);
    float *cur = new_data;

    for (unsigned int col = 0; col < m_num_cols; ++col)
    {
      for (unsigned int row = 0; row < m_num_rows; ++row)
      {
        *cur++ = data(row, col);
      }
    }

    unsigned int cols = m_num_cols;
    m_num_cols = m_num_rows;
    m_num_rows = cols;

    if (m_own_memory)
    {
      free(m_data);
      m_data = new_data;
    }
    else
    {
      for (unsigned int i = 0; i < m_num_elements; ++i) m_data[i] = new_data[i];
      free(new_data);
    }
  }

  return *this;
}

/** Computes a matrix that is the transposed of this matrix.
 * @return a matrix that is the transposed of this matrix
 */
Matrix
Matrix::get_transpose() const
{
  Matrix res(m_num_cols, m_num_rows);
  float *cur = res.get_data();

  for (unsigned int col = 0; col < m_num_cols; ++col)
  {
    for (unsigned int row = 0; row < m_num_rows; ++row)
    {
      *cur++ = data(row, col);
    }
  }
  return res;
}

/** Inverts the matrix.
 * The algorithm that is implemented for computing the inverse
 * of the matrix is the Gauss-Jordan-Algorithm. Hereby, the block-
 * matrix (A|I) consisting of the matrix to be inverted (A) and the
 * identity matrix (I) is transformed into (I|A^(-1)).
 * @return a reference to the matrix object which contains now the inverted matrix
 */
Matrix &
Matrix::invert()
{
  if (m_num_rows != m_num_cols)
  {
    throw fawkes::Exception("Matrix::invert(): Trying to compute inverse of non-quadratic matrix!");
  }

#ifdef HAVE_OPENCV
  CvMat cvmat = cvMat(m_num_rows, m_num_cols, CV_32FC1, m_data);
  cvInv(&cvmat, &cvmat, CV_LU);
#else
  Matrix i = Matrix::get_id(m_num_rows);

  // for each column...
  for (unsigned int col = 0; col < m_num_cols; ++col)
  {
    // ...multiply the row by the inverse of the element
    // on the diagonal...
    float factor = 1.f / data(col, col);
    i.mult_row(col, factor);
    this->mult_row(col, factor);

    // ...and subtract that row multiplied by the elements
    // in the current column from all other rows.
    for (unsigned int row = 0; row < m_num_rows; ++row)
    {
      if (row != col)
      {
        float factor2 = data(row, col);
        i.sub_row(row, col, factor2);
        this->sub_row(row, col, factor2);
      }
    }
  }

  overlay(0, 0, i);
#endif

  return *this;
}

/** Computes a matrix that is the inverse of this matrix.
 * @return a matrix that is the inverse of this matrix
 */
Matrix
Matrix::get_inverse() const
{
  Matrix res(*this);
  res.invert();

  return res;
}

/** Computes the determinant of the matrix.
 * @return the determinant
 */
float
Matrix::det() const
{
  if (m_num_rows != m_num_cols)
  {
    throw fawkes::Exception("Matrix::det(): The determinant can only be calculated for quadratic matrices.");
  }

#ifdef HAVE_OPENCV
  CvMat cvmat = cvMat(m_num_rows, m_num_cols, CV_32FC1, m_data);

  return (float)cvDet(&cvmat);
#else
  Matrix tmp_matrix(*this);
  float result = 1.f;

  // compute the upper triangular matrix
  for (unsigned int col = 0; col < m_num_cols; ++col)
  {
    float diag_elem = tmp_matrix.data(col, col);
    result *= diag_elem;

    // multiply n-th row by m(n,n)^{-1}
    tmp_matrix.mult_row(col, (1.f / diag_elem));
    for (unsigned int row = col + 1; row < m_num_rows; ++row)
    {
      tmp_matrix.sub_row(row, col, tmp_matrix.data(row, col));
    }
  }

  return result;
#endif
}

/** Returns a submatrix of the matrix.
 * @param row the row in the original matrix of the top-left element in the submatrix
 * @param col the column in the original matrix of the top-left element in the submatrix
 * @param num_rows the number of rows of the submatrix
 * @param num_cols the number of columns of the submatrix
 * @return the submatrix
 */
Matrix
Matrix::get_submatrix(unsigned int row, unsigned int col,
                      unsigned int num_rows, unsigned int num_cols) const
{
  if ((m_num_rows < row + num_rows) || (m_num_cols < col + num_cols))
  {
    throw fawkes::OutOfBoundsException("Matrix::get_submatrix(): The current matrix doesn't contain a submatrix of the requested dimension at the requested position.");
  }

  Matrix res(num_rows, num_cols);
  float *res_data = res.get_data();

  for (unsigned int r = 0; r < num_rows; ++r)
  {
    for (unsigned int c = 0; c < num_cols; ++c)
    {
      *res_data++ = data(row + r, col + c);
    }
  }

  return res;
}

/** Overlays another matrix over this matrix.
 * @param row the top-most row from which onwards the the elements are
 * exchanged for corresponding elements in the given matrix
 * @param col the left-most column from which onwards the the elements
 * are exchanged for corresponding elements in the given matrix
 * @param over the matrix to be overlaid
 */
void
Matrix::overlay(unsigned int row, unsigned int col, const Matrix &over)
{
  unsigned int max_row = std::min(m_num_rows, over.m_num_rows + row);
  unsigned int max_col = std::min(m_num_cols, over.m_num_cols + col);

  for (unsigned int r = row; r < max_row; ++r)
  {
    for (unsigned int c = col; c < max_col; ++c)
    {
      data(r, c) = over.data(r - row, c - col);
    }
  }
}

/** (Read-only) Access-operator.
 * With this operator it is possible to access a specific
 * element of the matrix. (First element is at (0, 0)
 * @param row the row of the element
 * @param col the column of the element
 * @return the value of the specified element
 */
/* Not True: To conform with the mathematical
 * fashion of specifying the elements of a matrix the top
 * left element of the matrix is accessed with (1, 1)
 * (i.e., numeration starts with 1 and not with 0).
 */
float
Matrix::operator()(unsigned int row, unsigned int col) const
{
  if (row >= m_num_rows || col >= m_num_cols)
  {
    throw fawkes::OutOfBoundsException("Matrix::operator() The requested element is not within the dimension of the matrix.");
  }

  return data(row, col);
}

/** (RW) Access operator.
 * see the read-only access operator for operational details
 * @param row the row of the element
 * @param col the column of the element
 * @return a reference to the specified element
 */
float &
Matrix::operator()(unsigned int row,
    unsigned int col)
{
  if (row >= m_num_rows || col >= m_num_cols)
  {
    throw fawkes::OutOfBoundsException("Matrix::operator() The requested element (%d, %d) is not within the dimension of the %dx%d matrix.");
  }

  return data(row, col);
}

/** Assignment operator.
 * Copies the data form the rhs Matrix to the lhs Matrix.
 * @param m the rhs Matrix
 * @return a reference to this Matrix
 */
Matrix &
Matrix::operator=(const Matrix &m)
{
  if (m_num_elements != m.m_num_elements)
  {
    if (!m_own_memory)
    {
      throw fawkes::OutOfBoundsException("Matrix::operator=(): The rhs matrix has not the same number of elements. This isn't possible if not self managing memory.");
    }

    m_num_elements = m.m_num_elements;
    free(m_data);
    m_data = (float*) malloc(sizeof(float) * m_num_elements);
  }

  m_num_rows = m.m_num_rows;
  m_num_cols = m.m_num_cols;

  for (unsigned int i = 0; i < m_num_elements; ++i) m_data[i] = m.m_data[i];

  return *this;
}

/** Matrix multiplication operator.
 * (Matrix)a.operator*((Matrix)b) computes a * b;
 * i.e., the 2nd matrix is right-multiplied to the 1st matrix
 * @param rhs the right-hand-side matrix
 * @return the product of the two matrices (a * b)
 */
Matrix
Matrix::operator*(const Matrix &rhs) const
{
  if (m_num_cols != rhs.m_num_rows)
  {
    throw fawkes::Exception("Matrix::operator*(...): Dimension mismatch: a %d x %d matrix can't be multiplied "
                            "with a %d x %d matrix.\n",
                            m_num_rows, m_num_cols, rhs.num_rows(), rhs.num_cols());
  }

  unsigned int res_rows = m_num_rows;
  unsigned int res_cols = rhs.m_num_cols;

  Matrix res(res_rows, res_cols);

  for (unsigned int r = 0; r < res_rows; ++r)
  {
    for (unsigned int c = 0; c < res_cols; ++c)
    {
      float t = 0.0f;

      for (unsigned int i = 0; i < m_num_cols; ++i)
      {
        t += data(r, i) * rhs.data(i, c);
      }

      res.data(r, c) = t;
    }
  }

  return res;
}

/** Combined matrix-multipliation and assignement operator.
 * @param rhs the right-hand-side Matrix
 * @return a reference to the Matrix that contains the result of the multiplication
 */
Matrix &
Matrix::operator*=(const Matrix &rhs)
{
  if (m_num_cols != rhs.m_num_rows)
  {
    throw fawkes::Exception("Matrix::operator*(...): Dimension mismatch: a %d x %d matrix can't be multiplied "
                            "with a %d x %d matrix.\n",
                            m_num_rows, m_num_cols, rhs.num_rows(), rhs.num_cols());
  }

  unsigned int res_rows     = m_num_rows;
  unsigned int res_cols     = rhs.m_num_cols;
  unsigned int res_num_elem = res_rows * res_cols;

  if (!m_own_memory && (m_num_elements != res_num_elem))
  {
    throw fawkes::Exception("Matrix::operator*=(): The resulting matrix has not the same number of elements. This doesn't work if not self managing memory.");
  }

  float *new_data = (float*) malloc(sizeof(float) * res_num_elem);
  float *cur = new_data;

  for (unsigned int r = 0; r < res_rows; ++r)
  {
    for (unsigned int c = 0; c < res_cols; ++c)
    {
      float t = 0.0f;

      for (unsigned int i = 0; i < m_num_cols; ++i)
      {
        t += data(r, i) * rhs.data(i, c);
      }

      *cur++ = t;
    }
  }

  if (m_own_memory)
  {
    free(m_data);
    m_data = new_data;
  }
  else
  {
    for (unsigned int i = 0; i < m_num_elements; ++i) m_data[i] = new_data[i];
    free(new_data);
  }

  return *this;
}

/** Multiply the matrix with given vector.
 * @param v a vector
 * @return the result of the matrix-vector multiplication
 */
Vector
Matrix::operator*(const Vector &v) const
{
  unsigned int cols = v.size();

  if (m_num_cols != cols)
  {
    throw fawkes::Exception("Matrix::operator*(...): Dimension mismatch: a %d x %d matrix can't be multiplied "
        "with a vector of length %d.\n", num_rows(), num_cols(), cols);
  }

  Vector res(m_num_rows);
  const float *vector_data = v.data_ptr();

  for (unsigned int r = 0; r < num_rows(); ++r)
  {
    float row_result = 0.f;

    for (unsigned int c = 0; c < cols; ++c)
    {
      row_result += data(r, c) * vector_data[c];
    }
    res[r] = row_result;
  }

  return res;
}

/** Mulitply every element of the matrix with the given scalar.
 * @param f a scalar
 * @return the result of the multiplication
 */
Matrix
Matrix::operator*(const float &f) const
{
  Matrix res(*this);
  float *data = res.get_data();

  for (unsigned int i = 0; i < res.m_num_elements; ++i)
  {
    data[i] *= f;
  }

  return res;
}

/** Combined scalar multiplication and assignment operator.
 * @param f a scalar
 * @return reference to the result
 */
Matrix &
Matrix::operator*=(const float &f)
{
  for (unsigned int i = 0; i < m_num_elements; ++i)
  {
    m_data[i] *= f;
  }

  return *this;
}

/** Divide every element of the matrix with the given scalar.
 * @param f a scalar
 * @return the result of the divison
 */
Matrix
Matrix::operator/(const float &f) const
{
  Matrix res(*this);
  float *data = res.get_data();

  for (unsigned int i = 0; i < res.m_num_elements; ++i)
  {
    data[i] /= f;
  }

  return res;
}

/** Combined scalar division and assignment operator.
 * @param f a scalar
 * @return reference to the result
 */
Matrix &
Matrix::operator/=(const float &f)
{
  for (unsigned int i = 0; i < m_num_elements; ++i)
  {
    m_data[i] /= f;
  }

  return *this;
}

/** Addition operator.
 * Adds the corresponding elements of the two matrices.
 * @param rhs the right-hand-side matrix
 * @return the resulting matrix
 */
Matrix
Matrix::operator+(const Matrix &rhs) const
{
  if ((m_num_rows != rhs.m_num_rows) || (m_num_cols != rhs.m_num_cols))
  {
    throw fawkes::Exception("Matrix::operator+(...): Dimension mismatch: a %d x %d matrix can't be added to a %d x %d matrix\n",
                            num_rows(), num_cols(), rhs.num_rows(), rhs.num_cols());
  }

  Matrix res(*this);
  const float *rhs_d = rhs.get_data();
  float *res_d = res.get_data();

  for (unsigned int i = 0; i < m_num_elements; ++i)
  {
    res_d[i] += rhs_d[i];
  }

  return res;
}

/**Add-assign operator.
 * @param rhs the right-hand-side matrix
 * @return a reference to the resulting matrix (this)
 */
Matrix &
Matrix::operator+=(const Matrix &rhs)
{
  if ((m_num_rows != rhs.m_num_rows) || (m_num_cols != rhs.m_num_cols))
  {
    throw fawkes::Exception("Matrix::operator+(...): Dimension mismatch: a %d x %d matrix can't be added to a %d x %d matrix\n",
                            num_rows(), num_cols(), rhs.num_rows(), rhs.num_cols());
  }

  const float *rhs_d = rhs.get_data();

  for (unsigned int i = 0; i < m_num_elements; ++i)
  {
    m_data[i] += rhs_d[i];
  }

  return *this;
}

/** Subtraction operator.
 * Subtracts the corresponding elements of the two matrices.
 * @param rhs the right-hand-side matrix
 * @return the resulting matrix
 */
Matrix
Matrix::operator-(const Matrix &rhs) const
{
  if ((num_rows() != rhs.num_rows()) || (num_cols() != rhs.num_cols()))
  {
    throw fawkes::Exception("Matrix::operator-(...): Dimension mismatch: a %d x %d matrix can't be subtracted from a %d x %d matrix\n",
        num_rows(), num_cols(), rhs.num_rows(), rhs.num_cols());
  }

  Matrix res(*this);

  const float *rhs_d = rhs.get_data();
  float *res_d = res.get_data();

  for (unsigned int i = 0; i < m_num_elements; ++i)
  {
    res_d[i] -= rhs_d[i];
  }

  return res;
}

/**Subtract-assign operator.
 * @param rhs the right-hand-side matrix
 * @return a reference to the resulting matrix (this)
 */
Matrix &
Matrix::operator-=(const Matrix &rhs)
{
  if ((num_rows() != rhs.num_rows()) || (num_cols() != rhs.num_cols()))
  {
    throw fawkes::Exception("Matrix::operator-=(...): Dimension mismatch: a %d x %d matrix can't be subtracted from a %d x %d matrix\n",
        num_rows(), num_cols(), rhs.num_rows(), rhs.num_cols());
  }

  const float *rhs_d = rhs.get_data();

  for (unsigned int i = 0; i < m_num_elements; ++i)
  {
    m_data[i] -= rhs_d[i];
  }

  return *this;
}

/** Comparison operator.
 * @param rhs the right-hand-side Matrix
 * @return true if every element of this matrix is equal to the
 * corresponding element of the other matrix
 */
bool
Matrix::operator==(const Matrix &rhs) const
{
  if ((num_rows() != rhs.num_rows()) || (num_cols() != rhs.num_cols()))
  {
    return false;
  }

  const float *rhs_d = rhs.get_data();

  for (unsigned int i = 0; i < m_num_elements; ++i)
  {
    if (m_data[i] != rhs_d[i]) return false;
  }

  return true;
}

/** Changes the matrix by multiplying a row with a factor.
 * @param row the row
 * @param factor the factor
 */
void
Matrix::mult_row(unsigned int row, float factor)
{
  if (row >= m_num_rows)
  {
    throw fawkes::OutOfBoundsException("Matrix::mult_row(...)", row, 0, m_num_rows);
  }

  for (unsigned int col = 0; col < m_num_cols; ++col)
  {
    data(row, col) *= factor;
  }
}

/** For two rows A and B and a factor f, A is changed to A - f*B.
 * @param row_a the row that is changed
 * @param row_b the row that is substracted from row_a
 * @param factor the factor by which every element of row_b is multiplied before it is
 *        substracted from row_a
 */
void
Matrix::sub_row(unsigned int row_a, unsigned int row_b, float factor)
{
  if (row_a >= m_num_rows)
  {
    throw fawkes::OutOfBoundsException("Matrix::sub_row(...) row_a", row_a, 0, m_num_rows);
  }
  if (row_b >= m_num_rows)
  {
    throw fawkes::OutOfBoundsException("Matrix::sub_row(...) row_b", row_b, 0, m_num_rows);
  }

  for (unsigned int col = 0; col < m_num_cols; ++col)
  {
    data(row_a, col) -= factor * data(row_b, col);
  }
}

/** Print matrix to standard out.
 * @param name a name that is printed before the content of the matrix (not required)
 * @param col_sep a string used to separate columns (defaults to '\\t')
 * @param row_sep a string used to separate rows (defaults to '\\n')
 */
void
Matrix::print_info(const char *name, const char *col_sep, const char *row_sep) const
{
  if (name)
  {
    printf("%s:\n", name);
  }

  for (unsigned int r = 0; r < num_rows(); ++r)
  {
    printf((r == 0 ? "[" : " "));
    for (unsigned int c = 0; c < num_cols(); ++c)
    {
      printf("%f", (*this)(r, c));
      if (c+1 < num_cols())
      {
        if (col_sep) printf("%s", col_sep);
        else printf("\t");
      }
    }
    if (r+1 < num_rows())
    {
      if (row_sep) printf("%s", row_sep);
      else printf("\n");
    }
    else printf("]\n\n");
  }
}

} // end namespace fawkes
