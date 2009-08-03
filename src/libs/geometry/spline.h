
/***************************************************************************
 *  spline.h - Cubic spline curve
 *
 *  Created: Tue Oct 07 18:57:42 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __GEOMETRY_SPLINE_H_
#define __GEOMETRY_SPLINE_H_

#include <geometry/transformable.h>
#include <geometry/hom_point.h>
#include <geometry/bezier.h>
#include <vector>

namespace fawkes {

class SplineDrawer;

class Spline : public Transformable
{
  friend class fawkes::SplineDrawer;
  
 public:
  Spline();
  Spline(const std::vector<HomPoint>& control_points);
  virtual ~Spline();

  void set_control_points(const std::vector<HomPoint>& control_points);
  void set_control_point(unsigned int i, const HomPoint& p);

  const std::vector<HomPoint>& get_control_points() const;
  const std::vector<Bezier>&   get_bezier_curves() const;

  HomPoint  eval(unsigned int bezier_index, float t);
  HomVector tangent(unsigned int bezier_index, float t);
  HomVector tangent(unsigned int point_index);
  std::vector<HomPoint> approximate(unsigned int num_subdivisions = 4);

 protected:
  // transformable
  virtual void register_primitives();
  virtual void post_transform();

 private:
  void construct_bezier_curves();

  std::vector<HomPoint> m_control_points;
  std::vector<Bezier>   m_bezier_curves;

  unsigned int m_num_subdivisions;
};

}

#endif /* __GEOMETRY_SPLINE_H_ */
