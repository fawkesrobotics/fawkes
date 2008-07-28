
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
#include <algorithm>

namespace fawkes {

/** @class Matrix matrix.h <geometry/matrix.h>
 * A general matrix class. It provides all the
 * operations that are commonly used with a matrix.
 * @author Daniel Beck
 */

/** @fn virtual inline unsigned int Matrix::num_rows() const
 * Return the number of rows in the Matrix
 * @return the number of rows
 */

/** @fn virtual inline unsigned int Matrix::num_cols() const
 * Return the number of columns in the Matrix
 * @return the number of columns
 */

/** Constructor.
 * @param num_rows number of rows
 * @param num_cols number of columns
 * @param data array containing elements of the matrix in row-order
 */
Matrix::Matrix(unsigned int num_rows,
               unsigned int num_cols,
               float *data )
{
	m_int_num_rows = num_rows;
	m_int_num_cols = num_cols;

	m_transposed = false;

	m_columns = new Vector *[m_int_num_cols];

	for (unsigned int i = 0; i < m_int_num_cols; ++i)
	{
		if ( data )
		{ m_columns[i] = new Vector(m_int_num_rows, &data[i * m_int_num_rows]); }
		else
		{ m_columns[i] = new Vector(m_int_num_rows); }
	}
}


/** Copy-constructor.
 * @param m another matrix
 */
Matrix::Matrix(const Matrix &m)
{
	m_int_num_rows = m.m_int_num_rows;
	m_int_num_cols = m.m_int_num_cols;

	m_transposed = m.m_transposed;

	m_columns = new Vector *[m_int_num_cols];

	for (unsigned int i = 0; i < m_int_num_cols; ++i)
	{
		m_columns[i] = new Vector(*(m.m_columns[i]));
	}
}


/** Destructor. */
Matrix::~Matrix()
{
	for (unsigned int i = 0; i < m_int_num_cols; ++i)
	{
		delete m_columns[i];
	}
	delete[] m_columns;
}


/** Determines the dimensions of the matrix.
 * @param num_cols pointer to an unsigned int to where the number of columns is copied to
 * @param num_rows pointer to an unsigned int to where the number of rows is copied to
 */
void
Matrix::size(unsigned int &num_rows,
             unsigned int &num_cols) const
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
	for (unsigned int row = 0; row < num_rows(); row++)
	{
		for (unsigned int col = 0; col < num_cols(); col++)
		{
			(*this)(row, col) = (row == col) ? 1.0 : 0.0;
		}
	}

	return *this;
}


/** Transposes the matrix.
 * Simply inverts m_transposed.
 * @return a reference to the matrix object now containing the transposed matrix
 */
Matrix &
Matrix::transpose()
{
	m_transposed = !m_transposed;

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
Matrix &
Matrix::invert()
{
	if (m_int_num_rows != m_int_num_cols)
	{
		printf("Matrix::invert(): Trying to compute inverse of non-quadratic matrix!\n");
		throw std::exception();
	}

	Matrix i(m_int_num_rows, m_int_num_cols);
	i.id();

	// for each column...
	for (unsigned int col = 0; col < num_cols(); col++)
	{
		// ...multiply the row by the inverse of the element
		// on the diagonal...
		float factor = 1.0f / (*this)(col, col);
		i.mult_row(col, factor);
		this->mult_row(col, factor);

		// ...and subtract that row multiplied by the elements
		// in the current column from all other rows.
		for (unsigned int row = 0; row < num_rows(); row++)
		{
			if (row != col)
			{
				float factor2 = (*this)(row, col);
				i.sub_row(row, col, factor2);
				this->sub_row(row, col, factor2);
			}
		}
	}

	*this = i;

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
	if (m_int_num_rows != m_int_num_cols)
	{
		printf("Matrix::det(): The determinant can only be calculated for nxn matrices.\n");
		throw std::exception();
	}

	Matrix m(*this);
	float result = 1.0f;

	// compute the upper triangular matrix
	for (unsigned int col = 0; col < num_cols(); col++)
	{
		float diag_elem = m(col, col);
		result *= diag_elem;

		// multiply n-th row by m(n,n)^{-1}
		m.mult_row( col, 1.0 / diag_elem );
		for (unsigned int row = col + 1; row < num_rows(); row++)
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
	if ( (row + num_rows) > this->num_rows() )
	{
		num_rows = this->num_rows() - row;
	}

	if ( (col + num_cols) > this->num_cols() )
	{
		num_cols = this->num_cols() - col;
	}

	Matrix m(num_rows, num_cols);

	for (unsigned int r = 0; r < num_rows; r++)
	{
		for (unsigned int c = 0; c < num_cols; c++)
		{
			m(r, c) = (*this)(row + r, col + c);
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
                const Matrix &m)
{
	unsigned int max_row = std::min(row + m.num_rows(), num_rows());
	unsigned int max_col = std::min(col + m.num_cols(), num_cols());

	for (unsigned int r = row; r < max_row; r++)
	{
		for (unsigned int c = col; c < max_col; c++)
		{
			(*this)(r, c) = m(r - row, c - col);
		}
	}
}


/** (Read-only) Access-operator.
 * With this operator it is possible to access a specific
 * element of the matrix.
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
Matrix::operator()(unsigned int row,
                   unsigned int col) const
{
	// TODO: sanity check

	if (m_transposed)
	{ return (*m_columns[row])[col]; }
	else
	{ return (*m_columns[col])[row]; }
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
Matrix &
Matrix::operator=(const Matrix &m)
{
	for (unsigned int i = 0; i < m_int_num_cols; ++i)
	{
		delete m_columns[i];
	}
	delete[] m_columns;

	m_int_num_rows = m.m_int_num_rows;
	m_int_num_cols = m.m_int_num_cols;

	m_transposed = m.m_transposed;

	m_columns = new Vector *[m_int_num_cols];

	for (unsigned int i = 0; i < m_int_num_cols; ++i)
	{
		m_columns[i] = new Vector(*(m.m_columns[i]));
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
Matrix::operator*(const Matrix &b) const
{
	const Matrix &a = (*this);

	if (a.num_cols() != b.num_rows())
	{
		printf("Matrix::operator*(...): Dimension mismatch: a %d x %d matrix can't be multiplied "
		       "with a %d x %d matrix.\n",
		       a.num_rows(), a.num_cols(), b.num_rows(), b.num_cols());
		throw std::exception();
	}

	unsigned int rows = a.num_rows();
	unsigned int cols = b.num_cols();

	Matrix result(rows, cols);

	for (unsigned int c = 0; c < cols; c++)
	{
		for (unsigned int r = 0; r < rows; r++)
		{
			float t = 0.0f;

			for (unsigned int i = 0; i < a.num_cols(); i++)
			{
				t += a(r, i) * b(i, c);
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
Matrix &
Matrix::operator*=(const Matrix &m)
{
	//TODO: more efficient direct mult
	*this = *this * m;

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

	if (num_cols() != cols)
	{
		printf("Matrix::operator*(...): Dimension mismatch: a %d x %d matrix can't be multiplied "
		       "with a vector of length %d.\n",
		       num_rows(), num_cols(), cols);
		throw std::exception();
	}

	Vector result(num_rows());

	for (unsigned int r = 0; r < num_rows(); ++r)
	{
		float row_result = 0.0;
		for (unsigned int c = 0; c < cols; ++c)
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
Matrix::operator*(const float &f) const
{
	Matrix result(*this);

	for (unsigned int i = 0; i < result.m_int_num_cols; ++i)
	{
		(*(result.m_columns[i])) *= f;
	}

	return result;
}

/** Combined scalar multiplication and assignment operator.
 * @param f a scalar
 * @return reference to the result
 */
Matrix &
Matrix::operator*=(const float &f)
{
	for (unsigned int i = 0; i < m_int_num_cols; ++i)
	{
		*(m_columns[i]) *= f;
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
	Matrix result(*this);

	for (unsigned int i = 0; i < result.m_int_num_cols; ++i)
	{
		(*(result.m_columns[i])) /= f;
	}

	return result;
}


/** Combined scalar division and assignment operator.
 * @param f a scalar
 * @return reference to the result
 */
Matrix &
Matrix::operator/=(const float &f)
{
	for (unsigned int i = 0; i < m_int_num_cols; ++i)
	{
		(*(m_columns[i])) /= f;
	}

	return *this;
}


/** Addition operator.
 * Adds the corresponding elements of the two matrices.
 * @param m the rhs matrix
 * @return the resulting matrix
 */
Matrix
Matrix::operator+(const Matrix &m) const
{
	if ((num_rows() != m.num_rows()) || (num_cols() != m.num_cols()))
	{
		printf("Matrix::operator+(...): Dimension mismatch: a %d x %d matrix can't be added to a %d x %d matrix\n",
		       num_rows(), num_cols(), m.num_rows(), m.num_cols());
		throw std::exception();
	}

	Matrix result(*this);

	for (unsigned int row = 0; row < num_rows(); row++)
	{
		for (unsigned int col = 0; col < num_cols(); col++)
		{
			result(row, col) += m(row, col);
		}
	}

	return result;
}


/**Add-assign operator.
 * @param m the rhs matrix
 * @return a reference to the resulting matrix (this)
 */
Matrix &
Matrix::operator+=(const Matrix &m)
{
	//TODO: more efficient direct add
	*this = *this + m;

	return *this;
}


/** Comparison operator.
 * @param m the rhs Matrix
 * @return true if every element of this matrix is equal to the
 * corresponding element of the other matrix
 */
bool
Matrix::operator==(const Matrix &m) const
{
	if ((num_rows() != m.num_rows()) || (num_cols() != m.num_cols()))
		return false;

	for (unsigned int r = 0; r < num_rows(); r++)
	{
		for (unsigned int c = 0; c < num_cols(); c++)
		{
			if ((*this)(r, c) != m(r, c))
				return false;
		}
	}

	return true;
}


/** Changes the matrix by multiplying a row with a factor.
 * @param row the row
 * @param factor the factor
 */
void
Matrix::mult_row(unsigned int row,
                 float factor)
{
	if (row > num_rows())
	{
		printf("Matrix::mult_row(...): Out of range: matrix has %d rows -- no %dth row.\n",
		       num_rows(), row);
		throw std::exception();
	}

	for (unsigned int col = 0; col < num_cols(); col++)
	{
		(*this)(row, col) *= factor;
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
	if ((row_a > num_rows()) || (row_b > num_rows()))
	{
		printf("Matrix::sub_row(...): Out of range: one of the arguments \"row_a\"=%d or \"row_b\"=%d is greater "
		       "than the number of rows (%d)\n",
		       row_a, row_b, num_rows());
		throw std::exception();
	}

	for (unsigned int col = 0; col < num_cols(); col++)
	{
		(*this)(row_a, col) -= factor * (*this)(row_b, col);
	}
}


/** Print matrix to standard out.
 * @param name a name that is printed before the content of the matrix (not required)
 * @param col_sep a string used to separate columns (defaults to '\\t')
 * @param row_sep a string used to separate rows (defaults to '\\n')
 */
void
Matrix::print_info(const char *name,
                   const char *col_sep, const char *row_sep) const
{
	if (name)
	{ printf("%s:\n", name); }

	for (unsigned int r = 0; r < num_rows(); ++r)
	{
		printf((r == 0 ? "[" : " "));
		for (unsigned int c = 0; c < num_cols(); ++c)
		{
			printf("%f", (*this)(r, c));
			if (c+1 < num_cols())
				printf(col_sep ? col_sep : "\t");
		}
		if (r+1 < num_rows())
			printf(row_sep ? row_sep : "\n");
		else
			printf("]\n\n");
	}
}

} // end namespace fawkes

