
/***************************************************************************
 *  rscf_circle.h - Header of circle shape model
 *                 using Randomized Stable Circle Fitting Algorithm
 *
 *  Generated: Fri Sep 09 2005 11:45:11
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *                   Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
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

#ifndef __FIREVISION_RSCF_CIRCLE_H_
#define __FIREVISION_RSCF_CIRCLE_H_

#include <vector>
#include <iostream>

#include <fvutils/types.h>
#include <models/shape/accumulators/fc_accum.h>
#include <models/shape/circle.h>

class ROI;

class RscfCircleModel: public ShapeModel
{
 private:
  static const int   MAX_NUM_ITERATION;
  static const float NEAR_DISTANCE;
  static const float MIN_RADIUS;
  static const float MAX_RADIUS;

 public:
  RscfCircleModel(void);
  virtual ~RscfCircleModel(void);

  std::string	getName(void) const {return std::string("RscfCircleModel");}
  int		parseImage(unsigned char* buffer, ROI *roi);
  int		getShapeCount(void) const;
  Circle*	getShape(int id) const;
  Circle*	getMostLikelyShape(void) const;
  
 private:
  std::vector< FittedCircle >             circles;
  std::vector< FittedCircle >::iterator   cit;

  std::vector< point_t >                  edge_pixels;
  std::vector< point_t >::iterator        epit;

};

#endif // __FIREVISION_RSCF_CIRCLE_H_
