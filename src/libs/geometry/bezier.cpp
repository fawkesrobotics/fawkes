
/***************************************************************************
 *  bezier.cpp - Bezier curve
 *
 *  Created: Mon Oct 06 15:14:53 2008
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

#include <geometry/bezier.h>
#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>

using namespace std;

namespace fawkes {

/** @class fawkes::Bezier <geometry/bezier.h>
 * A Bezier curve class.
 * @author Daniel Beck
 */

/** Constructor. */
Bezier::Bezier()
{
  m_de_casteljau_points = NULL;
  m_last_t = -1.0;
  m_num_subdivisions = 0;

  register_primitives();
}

/** Constructor.
 * @param control_points the control points for the Bezier curve
 */
Bezier::Bezier(const vector<HomPoint>& control_points)
  : m_control_points(control_points)
{
  m_num_control_points = m_control_points.size();

  m_de_casteljau_points = NULL;
  init_dclj_array();

  m_last_t = -1.0;
  m_num_subdivisions = 0;

  register_primitives();
}

/** Copy constructor.
 * @param b another Bezier curve
 */
Bezier::Bezier(const Bezier& b)
  : m_control_points(b.m_control_points)
{
  m_num_control_points = b.m_num_control_points;
  m_de_casteljau_points = NULL;
  init_dclj_array();

  m_last_t = -1.0;
  m_num_subdivisions = 0;
  
  clear_primitives();
  register_primitives();
}

/** Destructor. */
Bezier::~Bezier()
{
  for (unsigned int i = 0; i < m_dclj_array_size; ++i)
    { delete m_de_casteljau_points[i].first; }

  delete[] m_de_casteljau_points;
}

/** Set the control points.
 * @param control_points the new control points
 */
void
Bezier::set_control_points(const vector<HomPoint>& control_points)
{
  m_control_points.clear();
  m_control_points = control_points;

  m_num_control_points = m_control_points.size();

  m_last_t = -1.0;

  init_dclj_array();

  clear_primitives();
  register_primitives();
}

void
Bezier::init_dclj_array()
{
  m_dclj_array_size = m_num_control_points * (m_num_control_points + 1) / 2;
  m_dclj_array_size -= m_num_control_points;

  delete m_de_casteljau_points;
  m_de_casteljau_points = new pair<HomPoint*, bool>[m_dclj_array_size];

  for (unsigned int i = 0; i < m_dclj_array_size; ++i)
    {
      m_de_casteljau_points[i].first  = NULL;
      m_de_casteljau_points[i].second = false;
    }
}

/** Replace a specific control point.
 * @param index the index of the control point
 * @param control_point the replacement control point
 */
void
Bezier::set_control_point(unsigned int index, const HomPoint& control_point)
{
  m_control_points[index] = control_point;
  m_de_casteljau_points[index] = pair<HomPoint*, bool>( &(m_control_points[index]), true );

  m_last_t = -1.0;

  clear_primitives();
  register_primitives();
}

/** Get the control points.
 * @return a copy of the control points
 */
std::vector<HomPoint>
Bezier::get_control_points() const
{
  return m_control_points;
}

/** Get a specific control point.
 * @param i the index of the control point
 */
HomPoint
Bezier::get_control_point(unsigned int i) const
{
  if (i < m_num_control_points)
    { return m_control_points.at(i); }
  else
    { throw exception(); }
}

/** Get the degree of the polynom.
 * @return the degree of the polynom
 */
unsigned int
Bezier::degree() const
{
  return m_num_control_points - 1;
}

/** Evalutate the polynom for a given t
 * @param t a value between 0.0 and 1.0
 * @return the corresponding point on the curve
 */
HomPoint
Bezier::eval(float t)
{
  if ( t < 0 || t > 1)
    { throw exception(); }

  return de_casteljau(m_num_control_points - 1, 0, t); 
}

/** Compute the tangent vector at position t.
 * @param t the curve parameter
 * @return the tangent vector
 */
HomVector
Bezier::tangent_at_t(float t)
{
  HomVector v;
  HomPoint b0 = de_casteljau(m_num_control_points - 2, 0, t);
  HomPoint b1 = de_casteljau(m_num_control_points - 2, 1, t);
  v = b1 - b0;

  return v;
}

/** Compute the tangent vector at the specified control point.
 * @param index the index of the control point
 * @return the tangent vector
 */
HomVector
Bezier::tangent_at_point(unsigned int index)
{
  float t;
  if (index > m_num_control_points)
    { t = 1.0; }
  else
    { t = index / (float) m_num_control_points; }

  return tangent_at_t(t);  
}

/** Subdivide the curve into two polynome of the same degree.
 * @param t determines the point where the curve is divided
 * @param c the Bezier for the part [0, t]
 * @param d the Bezier for the part [t, 1]
 */
void
Bezier::subdivide(float t, Bezier& c, Bezier& d)
{
  if ( t < 0 || t > 1 )
    { throw exception(); }

  vector<HomPoint> control_points;

  for (unsigned k = 0; k < m_num_control_points; ++k)
    {
      HomPoint p = de_casteljau(k, 0, t);
      control_points.push_back(p);
    }

  c.set_control_points(control_points);
  control_points.clear();

  for (unsigned i = 0; i < m_num_control_points; ++i)
    {
      unsigned int k = m_num_control_points - i - 1;
      HomPoint p = de_casteljau(k, i, t);
      control_points.push_back(p);
    }
  
  d.set_control_points(control_points);
}

/** Approximate the curve with points.
 * @param num_subdivisions the number of subdivisions that is performed
 * @return the point approximating the curve
 */
const vector<HomPoint>&
Bezier::approximate(unsigned int num_subdivisions)
{
  if (m_num_subdivisions == num_subdivisions)
    { return m_approximation; }

  vector<Bezier> b1;
  vector<Bezier> b2;

  b1.push_back( *this );

  for (unsigned int i = 0; i < num_subdivisions; ++i)
    {
      b2.clear();

      for ( vector<Bezier>::iterator iter = b1.begin();
	    iter != b1.end();
	    ++iter )
	{
	  Bezier c, d;
	  iter->subdivide(0.5, c, d);
	  b2.push_back(c);
	  b2.push_back(d);
	}

      b1.clear();
      b1 = b2;
    }

  for ( vector<Bezier>::iterator bit = b2.begin();
	bit != b2.end();
	++bit )
    {
      vector<HomPoint> points = bit->get_control_points();

      vector<HomPoint>::iterator pit = points.begin();

      if ( bit != b2.begin() )
	// skip first control point for all Bezier curves except for
	// the first one
	{ ++pit; }

      for ( vector<HomPoint>::iterator iter = pit;
	    iter != points.end();
	    ++iter )
	{ m_approximation.push_back( *iter); }
    }

  m_num_subdivisions = num_subdivisions;
  
  return m_approximation;
}

HomPoint
Bezier::de_casteljau(unsigned int k, unsigned int i, float t)
{
  if (0 == k)
    { return m_control_points.at(i); }

  if (m_last_t != t)
    {
      for ( unsigned int j = 0;
	    j < m_dclj_array_size;
	    ++j )
	{ 
	  delete m_de_casteljau_points[j].first;

	  m_de_casteljau_points[j].first  = NULL;
	  m_de_casteljau_points[j].second = false;
	}

      m_last_t = t;
    }

  unsigned int index = get_dclj_array_index(k, i);
  
  if ( m_de_casteljau_points[index].second )
    { return *( m_de_casteljau_points[index].first ); }
  else
    {
      HomPoint* p = new HomPoint();
      *p = de_casteljau(k-1, i, t) * (1.0 - t) + de_casteljau(k-1, i+1, t) * t;
      m_de_casteljau_points[index] = pair<HomPoint*, bool>(p, true);
      return *p;
    }
}

unsigned int
Bezier::get_dclj_array_index(unsigned int k, unsigned int i) const
{
  unsigned int index = 0;

  for (unsigned int j = 0; j < k; ++j)
    { index += m_num_control_points - j; }

  index += i;
  index -= m_num_control_points;

  return index;
}

void
Bezier::register_primitives()
{
  vector<HomPoint>::iterator iter;
  for ( iter  = m_control_points.begin();
	iter != m_control_points.end();
	++iter )
    {
      HomPoint& p = *iter;
      add_primitive( &p );
    }
}

void
Bezier::post_transform()
{
  m_last_t = -1.0;
}

} // end namespace fawkes
