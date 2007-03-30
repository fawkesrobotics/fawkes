
/***************************************************************************
 *  histogram.h - Header for 2D histograms
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

#include <fvutils/types.h>
#include <iostream>

class Histogram2D
{

 public:
  Histogram2D(unsigned int width, unsigned int height, unsigned int num_undos = 1);
  ~Histogram2D();

  void           operator+=(point_t *p);
  void           operator+=(point_t  p);

  unsigned int * getHistogram();
  unsigned int   getValue(unsigned int x, unsigned int y);
  void           setValue(unsigned int x, unsigned int y, unsigned int value);
  void           reset();
  unsigned int   getMedian();
  unsigned int   getAverage();

  unsigned int   getSum() const;

  void           resetUndo();
  void           undo();
  unsigned int   switchUndo( unsigned int undo_id );
  unsigned int   getNumUndos();

  void           printToStream(std::ostream &s);
  void           save(const char * filename);
  bool           load(const char * filename);

 private:
  unsigned int  width;
  unsigned int  height;
  unsigned int *histogram;
  unsigned int  number_of_values;

  unsigned int **undo_overlay;
  unsigned int  *undo_num_vals;
  unsigned int   undo_num;
  unsigned int   undo_current;

};



#endif
