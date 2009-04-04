
/***************************************************************************
 *  matrix.h - A matrix class
 *
 *  Created: Wed Sep 26 14:28:01 2007
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

#ifndef __GEOMETRY_MATRIX_H_
#define __GEOMETRY_MATRIX_H_

namespace fawkes {

class Vector;

class Matrix
{
public:
  Matrix(unsigned int num_rows = 0, unsigned int num_cols = 0,
	 float *data = 0, bool manage_own_memory = true);
  Matrix(const Matrix &tbc);
  ~Matrix();

  void size(unsigned int &num_rows, unsigned int &num_cols) const;
  inline unsigned int num_rows() const { return m_num_rows; }
  inline unsigned int num_cols() const { return m_num_cols; }

  Matrix &id();
  static Matrix get_id(unsigned int size, float *data_buffer = 0);
  static Matrix get_diag(unsigned int size, float value, float *data_buffer = 0);

  Matrix &transpose();
  Matrix get_transpose() const;

  Matrix &invert();
  Matrix get_inverse() const;

  float det() const;

  const float* get_data() const { return m_data; }
  float* get_data() { return m_data; }

  Matrix get_submatrix(unsigned int row, unsigned int col,
                         unsigned int num_rows, unsigned int num_cols) const;

  void overlay(unsigned int row, unsigned int col, const Matrix &m);

  float operator()(unsigned int row, unsigned int col) const;
  float &operator()(unsigned int row, unsigned int col);

  Matrix& operator=(const Matrix &rhs);

  Matrix  operator*(const Matrix &rhs) const;
  Matrix& operator*=(const Matrix &rhs);

  Vector operator*(const Vector &cv) const;

  Matrix  operator*(const float &f) const;
  Matrix& operator*=(const float &f);

  Matrix  operator/(const float &f) const;
  Matrix& operator/=(const float &f);

  Matrix  operator+(const Matrix &rhs) const;
  Matrix& operator+=(const Matrix &rhs);

  Matrix  operator-(const Matrix &rhs) const;
  Matrix& operator-=(const Matrix &rhs);

  bool operator==(const Matrix &rhs) const;

  void print_info(const char *name = 0, const char *col_sep = 0,
                  const char *row_sep = 0) const;

private:
  void mult_row(unsigned int row, float factor);
  void sub_row(unsigned int row_a, unsigned int row_b, float factor);
  inline float  data(unsigned int row, unsigned int col) const
  {
    return m_data[row * m_num_cols + col];
  }
  inline float& data(unsigned int row, unsigned int col)
  {
    return m_data[row * m_num_cols + col];
  }

private:
  float *m_data;

  unsigned int m_num_rows;
  unsigned int m_num_cols;
  unsigned int m_num_elements;

  bool m_own_memory;
};

} // end namespace fawkes

#endif /* __GEOMETRY_MATRIX_H_ */
