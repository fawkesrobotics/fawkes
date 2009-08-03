
/***************************************************************************
 *  drawing_manipulator.h - Manipulates things like point size, line
 *  width, etc.
 *
 *  Created: Fri Oct 10 18:09:38 2008
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

#ifndef __GEOMETRY_DRAWING_MANIPULATOR_H_
#define __GEOMETRY_DRAWING_MANIPULATOR_H_

#include <geometry/gtk/geom_drawer.h>

namespace fawkes {

class DrawingManipulator : public GeomDrawer
{
 public:
  DrawingManipulator();
  virtual ~DrawingManipulator();

  enum Color
  {
    BLACK,     /**< black */
    WHITE,     /**< white */
    RED,       /**< red */
    GREEN,     /**< green */
    BLUE       /**< blue */
  };

  void integrate(const DrawingManipulator* m);

  void  set_line_width(float w);
  float get_line_width() const;

  void  set_point_size(float s);
  float get_point_size() const;

  void set_color(Color c);
  void set_color(float r, float g, float b);
  void get_color(float& r, float& g, float& b) const;

  virtual void draw(Cairo::RefPtr<Cairo::Context>& context);
   
 private:
  float m_line_width;
  float m_point_size;
  float m_color_r;
  float m_color_g;
  float m_color_b;

  bool m_line_width_set;
  bool m_point_size_set;
  bool m_color_set;
};

DrawingManipulator* set_line_width(float w);
DrawingManipulator* set_point_size(float s);
DrawingManipulator* set_color(float r, float g, float b);
DrawingManipulator* set_color(DrawingManipulator::Color c);

} // end namespace fawkes

#endif /* __GEOMETRY_DRAWING_MANIPULATOR_H_ */
