
/***************************************************************************
 *  bezier.h - Bezier curve
 *
 *  Created: Mon Oct 06 14:52:57 2008
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

#ifndef __GEOMETRY_BEZIER_H_
#define __GEOMETRY_BEZIER_H_

#include <geometry/transformable.h>
#include <vector>

namespace fawkes {
class HomPoint;
class HomVector;

class Bezier : public Transformable
{
 public:
  Bezier();
  Bezier(const std::vector<HomPoint>& control_points);
  Bezier(const Bezier& b);
  ~Bezier();

  void set_control_points(const std::vector<HomPoint>& control_points);
  void set_control_point(unsigned int index, const HomPoint& control_point);
  
  std::vector<HomPoint> get_control_points() const;
  HomPoint              get_control_point(unsigned int i) const;

  unsigned int degree() const;
 
  HomPoint  eval(float t);
  HomVector tangent_at_t(float t);
  HomVector tangent_at_point(unsigned int index);
  void      subdivide(float t, Bezier& c, Bezier& d);
  const std::vector<HomPoint>& approximate(unsigned int num_subdivisions = 4);

 protected:
  // transformable
  virtual void register_primitives();
  virtual void post_transform();

 private:
  void init_dclj_array();
  unsigned int get_dclj_array_index(unsigned int k, unsigned int i) const;

  std::vector<HomPoint> m_control_points;
  std::vector<HomPoint> m_approximation;
  unsigned int m_num_subdivisions;

  HomPoint de_casteljau(unsigned int k, unsigned int i, float t);
  
  std::pair<HomPoint*, bool>* m_de_casteljau_points;
  unsigned int m_dclj_array_size;

  unsigned int m_num_control_points;

  float m_last_t;
};

} // end namespace fawkes

#endif /* __GEOMETRY_BEZIER_H_ */
