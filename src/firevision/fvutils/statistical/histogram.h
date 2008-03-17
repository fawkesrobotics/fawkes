
/***************************************************************************
 *  histogram.h - Header for histograms
 *
 *  Generated: Tue Jun 14 11:09:27 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_UTILS_HISTOGRAM_H_
#define __FIREVISION_UTILS_HISTOGRAM_H_

#include <fvutils/base/types.h>
#include <iostream>

class Histogram
{
 public:
  Histogram(unsigned int width, unsigned int height,
	    unsigned int depth = 1, unsigned int num_undos = 1);
  ~Histogram();

  void           operator+=(point_t *p);
  void           operator+=(point_t  p);

  unsigned int * get_histogram();
  unsigned int   get_value(unsigned int x, unsigned int y);
  unsigned int   get_value(unsigned int x, unsigned int y, unsigned int z);
  void           set_value(unsigned int x, unsigned int y, unsigned int value);
  void           set_value(unsigned int x, unsigned int y, unsigned int z, unsigned int value);
  void           inc_value(unsigned int x, unsigned int y, unsigned int z = 0);
  void           add(unsigned int x, unsigned int y, unsigned int z, unsigned int value);
  void           sub(unsigned int x, unsigned int y, unsigned int z, unsigned int value);
  void           reset();
  unsigned int   get_median();
  unsigned int   get_average();

  unsigned int   get_sum() const;

  void           reset_undo();
  void           undo();
  unsigned int   switch_undo( unsigned int undo_id );
  unsigned int   get_num_undos();

  void           print_to_stream(std::ostream &s);
  void           save(const char * filename, bool formatted_output = false);
  bool           load(const char * filename);

 private:
  unsigned int  width;
  unsigned int  height;
  unsigned int  depth;
  unsigned int  dimension;
  unsigned int  histogram_size;
  unsigned int *histogram;
  unsigned int  number_of_values;

  unsigned int **undo_overlay;
  unsigned int  *undo_num_vals;
  unsigned int   undo_num;
  unsigned int   undo_current;
};



#endif
