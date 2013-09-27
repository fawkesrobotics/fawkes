
/***************************************************************************
 *  rcd_circle.h - Header of circle shape model
 *                 using Random Circle Detection Algorithm
 *
 *  Created: Thu May 16 00:00:00 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *                   Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
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

#ifndef __FIREVISION_MODELS_SHAPE_RCD_CIRCLE_H_
#define __FIREVISION_MODELS_SHAPE_RCD_CIRCLE_H_

#include <vector>
#include <iostream>

#include <utils/math/types.h>
#include <fvutils/base/types.h>
#include <fvmodels/shape/circle.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ROI;

class RcdCircleModel: public ShapeModel
{
 private:
  std::vector<Circle> m_Circles;
 public:

  RcdCircleModel(unsigned int max_failures       = 300,
                 unsigned int min_pixels         =  20,
                 unsigned int min_interpix_dist  =  10,
                 unsigned int max_dist_p4        =   2,
                 unsigned int max_dist_a         =  10,
                 float        hw_ratio           =   0.6,
                 float        hollow_rate        =   0.f,
                 float        max_time           =   0.01
                 );
  virtual ~RcdCircleModel(void);

  std::string   getName(void) const {return std::string("RcdCircleModel");}
  int           parseImage(unsigned char* buffer, ROI *roi);
  int           getShapeCount(void) const;
  Circle*       getShape(int id) const;
  Circle*       getMostLikelyShape(void) const;

 private:
  /** Calculate circle from three points
   */
  void          calcCircle( const fawkes::upoint_t& p1,
                            const fawkes::upoint_t& p2,
                            const fawkes::upoint_t& p3,
                            center_in_roi_t& center,
                            float& radius);



  int    diff_sec;
  int    diff_usec;
  float  f_diff_sec;

  unsigned int  RCD_MAX_FAILURES;
  unsigned int  RCD_MIN_PIXELS;
  unsigned int  RCD_MIN_INTERPIX_DIST;
  unsigned int  RCD_MAX_DIST_P4;
  unsigned int  RCD_MAX_DIST_A;
  float         RCD_HW_RATIO;
  float         RCD_MAX_TIME;
  float         RCD_ROI_HOLLOW_RATE;

};

} // end namespace firevision

#endif

