
/***************************************************************************
 *  radial.h - Scanline model implementation: radial
 *
 *  Generated: Tue Jul 19 12:05:31 2005
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

#ifndef __FIREVISION_SCANLINE_RADIAL_H_
#define __FIREVISION_SCANLINE_RADIAL_H_

#include "models/scanlines/scanlinemodel.h"
#include <fvutils/types.h>

class ScanlineRadial : public ScanlineModel
{

 public:

  ScanlineRadial(unsigned int width, unsigned int height,
		 unsigned int center_x, unsigned int center_y,
		 unsigned int radius_increment, unsigned int step,
		 unsigned int dead_radius = 0
		 );

  point_t  operator*();
  point_t* operator->();
  point_t* operator++();
  point_t* operator++(int); 

  bool          finished();
  void          reset();
  const char *  getName();
  unsigned int  getMargin();

  virtual void  setRobotPose(float x, float y, float ori) {}
  virtual void  setPanTilt(float pan, float tilt) {}

 private:

  void simpleBubbleSort(unsigned int array[], unsigned int num_elements);

  unsigned int width;
  unsigned int height;
  unsigned int center_x;
  unsigned int center_y;
  unsigned int radius_increment;
  unsigned int step;
  unsigned int current_radius;
  unsigned int max_radius;
  unsigned int dead_radius;

  point_t coord;
  point_t tmp_coord;

  unsigned int sector;

  bool done;

  int x;
  int y;
  int tmp_x;
  int tmp_y;

};

#endif
