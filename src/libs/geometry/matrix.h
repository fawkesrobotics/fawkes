
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
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __MATRIX_H_
#define __MATRIX_H_

#include <iostream>

class Matrix
{
  friend std::ostream& operator<<(std::ostream &ostr, const Matrix& m);

 public:
  Matrix( unsigned int num_rows,
	  unsigned int num_cols );
  Matrix(const Matrix& m);
  virtual ~Matrix();

  void get_dimensions( unsigned int* num_rows,
		       unsigned int* num_cols ) const;

  Matrix& id();

  Matrix& transpose();
  Matrix get_transpose() const;

  Matrix& invert();
  Matrix get_invert() const;

  float det() const;

  Matrix submatrix( unsigned int row,
		    unsigned int col,
		    unsigned int num_rows,
		    unsigned int num_cols ) const;

  void overlay( unsigned int row,
		unsigned int col,
		const Matrix& m );

  float operator()( unsigned int row,
		    unsigned int col ) const;
  float& operator()( unsigned int row,
		     unsigned int col );
  Matrix& operator=(const Matrix& m);
  Matrix operator*(const Matrix& m) const;
  Matrix& operator*=(const Matrix& m);
  Matrix operator+(const Matrix& m) const;
  Matrix& operator+=(const Matrix& m);
  bool operator==(const Matrix& m) const;

  void print_to_stream(std::ostream& ostr) const;

 private:
  void mult_row( unsigned int row,
		 double factor );
  void sub_row( unsigned int row_a,
		unsigned int row_b,
		float factor );
  
  float* mData;
  unsigned int mNumRows;
  unsigned int mNumCols;  
};

#endif /* __MATRIX_H_ */
