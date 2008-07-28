
/***************************************************************************
 *  matrix.h - A matrix class
 *
 *  Created: Wed Sep 26 14:28:01 2007
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

#ifndef __GEOMETRY_MATRIX_H_
#define __GEOMETRY_MATRIX_H_

namespace fawkes {

class Vector;

class Matrix
{
 public:
	Matrix(unsigned int num_rows = 0,
		unsigned int num_cols = 0,
		float *data = 0);
	Matrix(const Matrix &m);
	virtual ~Matrix();

	virtual void size(unsigned int &num_rows,
		unsigned int &num_cols) const;
	virtual inline unsigned int num_rows() const
	{ return m_transposed ? m_int_num_cols : m_int_num_rows; }
	virtual inline unsigned int num_cols() const
	{ return m_transposed ? m_int_num_rows : m_int_num_cols; }

	virtual Matrix &id();

	virtual Matrix &transpose();
	virtual Matrix  get_transpose() const;

	virtual Matrix &invert();
	virtual Matrix  get_inverse() const;

	virtual float det() const;

	virtual Matrix get_submatrix(unsigned int row,
		unsigned int col,
		unsigned int num_rows,
		unsigned int num_cols) const;

	virtual void overlay(unsigned int row,
		unsigned int col,
		const Matrix &m);

	virtual float  operator()(unsigned int row,
		unsigned int col) const;
	virtual float &operator()(unsigned int row,
		unsigned int col);

	virtual Matrix &operator=(const Matrix &m);

	virtual Matrix  operator*(const Matrix &m) const;
	virtual Matrix &operator*=(const Matrix &m);

	virtual Vector operator*(const Vector &cv) const;

	virtual Matrix  operator*(const float &f) const;
	virtual Matrix &operator*=(const float &f);

	virtual Matrix  operator/(const float &f) const;
	virtual Matrix &operator/=(const float &f);

	virtual Matrix  operator+(const Matrix &m) const;
	virtual Matrix &operator+=(const Matrix &m);

	virtual bool operator==(const Matrix &m) const;

	virtual void print_info(const char *name = 0,
		const char *col_sep = 0,
		const char *row_sep = 0) const;

 private:
	virtual void mult_row(unsigned int row,
		float factor );
	virtual void sub_row( unsigned int row_a,
		unsigned int row_b,
		float factor );

 private:
	Vector **m_columns;

	/* Internal number of rows / cols of the stored data - 
	   Needed because of the unfortunate m_transposed concept */
	unsigned int m_int_num_rows;
	unsigned int m_int_num_cols;

	bool m_transposed;
};

} // end namespace fawkes

#endif /* __GEOMETRY_MATRIX_H_ */
