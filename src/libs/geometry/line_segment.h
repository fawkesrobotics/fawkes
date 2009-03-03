
/***************************************************************************
 *  line_segment.h - A line segment
 *
 *  Created: Thu Oct 02 16:47:39 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __GEOMETRY_LINE_SEGMENT_H_
#define __GEOMETRY_LINE_SEGMENT_H_

#include <geometry/transformable.h>
#include <geometry/printable.h>
#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>

namespace fawkes {

class LineSegment 
  : public Transformable,
    public Printable
    
{
 public:
  LineSegment(const HomPoint& a, const HomPoint& b);
  LineSegment(const HomPoint& p, const HomVector& v);
  LineSegment(const LineSegment& l);
  virtual ~LineSegment();

  float length() const;

  const HomPoint& p1() const;
  const HomPoint& p2() const;

 protected:
  virtual void register_primitives();
  virtual void post_transform();
  virtual std::ostream& print(std::ostream& stream) const;

 private:
  HomPoint m_p1;
  HomPoint m_p2;
};

} // end namespace fawkes

#endif /* __GEOMETRY_LINE_SEGMENT_H_ */
