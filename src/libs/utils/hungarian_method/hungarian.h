
/***************************************************************************
 *  hungarian.h - Hungarian Method
 *
 *  Created: Thu Apr 18 17:09:58 2013
 *  Copyright  2004  Cyrill Stachniss
 *             2008  Masrur Doostdar
 *             2008  Stefan Schiffer
 *             2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_HUNGARIAN_METHOD_HUNGARIAN_H_
#define __UTILS_HUNGARIAN_METHOD_HUNGARIAN_H_

#define HUNGARIAN_NOT_ASSIGNED 0 
#define HUNGARIAN_ASSIGNED 1

#define HUNGARIAN_MODE_MINIMIZE_COST 0
#define HUNGARIAN_MODE_MAXIMIZE_UTIL 1

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/// @cond INTERNAL
typedef struct {
  int num_rows;
  int num_cols;
  int** cost;
  int** assignment;  
} hungarian_problem_t;
/// @endcond

class HungarianMethod
{
 public:
  HungarianMethod();
  ~HungarianMethod();

  int init( int** cost_matrix, 
	    int rows, int cols, int mode);
  
  void free();

  void solve();

  bool is_available();

  int  get_column_assignment( const int & col );
  int  get_row_assignment( const int & row );
  int* get_assignment(int & size);

  int** array_to_matrix(int* m, int rows, int cols);

  void print_assignment();
  void print_cost_matrix();
  void print_status();

  /** our problem instance member.  */
  hungarian_problem_t * p;

 protected:
  void print_matrix( int** C, int rows, int cols );

 private:
  bool available_;
  int  num_cols_;
  int  num_rows_;

  int * col_mates_;
  int * row_mates_;
};

} // end namespace fawkes

#endif



