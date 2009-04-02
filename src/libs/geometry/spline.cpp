
/***************************************************************************
 *  spline.cpp - Cubic spline curve
 *
 *  Created: Tue Oct 07 19:03:51 2008
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

#include <geometry/spline.h>
#include <geometry/hom_vector.h>
#include <cmath>

using namespace std;

namespace fawkes {

/** @class fawkes::Spline <geometry/spline.h>
 * A spline made up of cubic Bezier curves.
 * @author Daniel Beck
 */

/** Constructor. */
Spline::Spline()
{
  m_num_subdivisions = 0;
}

/** Constructor.
 * @param control_points the control points of the spline
 */
Spline::Spline(const vector<HomPoint>& control_points)
  : m_control_points(control_points)
{
  m_num_subdivisions = 0;

  register_primitives();
  construct_bezier_curves();
}

/** Destructor. */
Spline::~Spline()
{
}

/** Set the control points.
 * @param control_points the new control points
 */
void
Spline::set_control_points(const vector<HomPoint>& control_points)
{
  m_control_points = control_points;

  clear_primitives();
  register_primitives();
  construct_bezier_curves();
}

/** Set a specific control point.
 * @param i the index of the control point
 * @param point the replacement control point
 */
void
Spline::set_control_point(unsigned int i, const HomPoint& point)
{
  m_control_points[i] = point;

  clear_primitives();
  register_primitives();
  construct_bezier_curves();
}

/** Get the control points.
 * @return const reference to the current control points
 */
const vector<HomPoint>&
Spline::get_control_points() const
{
  return m_control_points;
}

/** Get the Bezier curves.
 * @return const reference to the current Bezier curves
 */
const vector<Bezier>&
Spline::get_bezier_curves() const
{
  return m_bezier_curves;
}

/** Get a point on the curve for a specified segment and value t.
 * @param bezier_index the index of the curve
 * @param t a value between 0.0 and 1.0
 * @return a point on the i-th curve for the parameter t
 */
HomPoint
Spline::eval(unsigned int bezier_index, float t)
{
  return m_bezier_curves[bezier_index].eval(t);
}

/** Compute the tangent vector at position t of the i-th Bezier curve.
 * @param bezier_index the index of the Bezier patch
 * @param t the curve parameter t
 * @return the tangent vector
 */
HomVector
Spline::tangent(unsigned int bezier_index, float t)
{
  return m_bezier_curves[bezier_index].tangent_at_t(t);
}

/** Compute the tangent vector at the specified approximation point.
 * The range of the index is determined by number of subidivisions
 * that have been performed during the last approximation.
 * @param point_index index of the approximation point
 * @return tangent vector
 */
HomVector
Spline::tangent(unsigned int point_index)
{
  unsigned int points_per_bezier = (unsigned int) rint( powf(2.0, m_num_subdivisions) ) * 3;
  unsigned int points_total = m_bezier_curves.size() * (points_per_bezier - 1) + 1;

  if (point_index > points_total)
    { return m_bezier_curves.back().tangent_at_t(1.0); }
  else
    {
      unsigned int bezier_index = point_index / points_per_bezier;
      unsigned int index = point_index - bezier_index * points_per_bezier;
      
      return m_bezier_curves[bezier_index].tangent_at_t( index / (float) points_per_bezier );
    }
}

/** Get linear approximation of the curve.
 * Instead of evaluating the Bezier polynoms at constant intervals the
 * Bezier curves are subdivided and the control points are taken as an
 * approximation. This is more efficient and yields better
 * results. The control points converge to the curve quadratically in
 * the number of subdivisions.
 * @param num_subdivisions the number of subdivision that shall be
 * performed.
 * @return points approximating the curve
 */
vector<HomPoint>
Spline::approximate(unsigned int num_subdivisions)
{
  vector<HomPoint> approximation;

  for ( vector<Bezier>::iterator bit = m_bezier_curves.begin();
	bit != m_bezier_curves.end();
	++bit )
    {
      vector<HomPoint> points = bit->approximate(num_subdivisions);
      vector<HomPoint>::iterator pit = points.begin();

      if ( bit != m_bezier_curves.begin() )
	// skip first control point for all Bezier curves except for
	// the first one
	{ ++pit; }

      for ( vector<HomPoint>::iterator iter = pit;
	    iter != points.end();
	    ++iter )
	{
	  approximation.push_back( *iter );
	}
    }

  m_num_subdivisions = num_subdivisions;

  return approximation;
}

void
Spline::register_primitives()
{
  vector<HomPoint>::iterator iter;
  for ( iter  = m_control_points.begin();
	iter != m_control_points.end();
	++iter )
    {
      HomCoord& c = *iter;
      add_primitive( &c );
    }
}

void
Spline::post_transform()
{
  construct_bezier_curves();
}

void
Spline::construct_bezier_curves()
{
  m_bezier_curves.clear();

  if ( 0 == m_control_points.size() )
  { return; }

  vector<HomPoint>::iterator i    = m_control_points.begin();
  vector<HomPoint>::iterator prev = i;
  vector<HomPoint>::iterator cur  = ++i;
  vector<HomPoint>::iterator next = ++i;

  HomPoint cp1 = (*prev);

  while ( cur != m_control_points.end() )
    {
      HomPoint cp2;
      HomPoint cp3;
      HomPoint cp4;

      HomVector v;

      v = (*cur) - (*prev);
      
      cp2 = (*prev) + v * (1.0 / 3.0);
      cp3 = (*prev) + v * (2.0 / 3.0);

      if ( next == m_control_points.end() )
	{ cp4 = *cur; }
      else
	{
	  HomPoint t;

	  v = (*next) - (*cur);
 	  t = (*cur) + v * (1.0 / 3.0);
	  cp4 = cp3 + (t - cp3) * 0.5;
	}

      vector<HomPoint> control_points;
      control_points.push_back(cp1);
      control_points.push_back(cp2);
      control_points.push_back(cp3);
      control_points.push_back(cp4);
      
      m_bezier_curves.push_back( Bezier(control_points) );

      cp1 = cp4;
      ++prev;
      ++cur;
      ++next;
    }
}

} // end namespace fawkes
