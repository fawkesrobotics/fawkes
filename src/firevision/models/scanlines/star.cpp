
/***************************************************************************
 *  star.h - Starlike scanline model
 *
 *  Generated: Mon Nov 05 10:06:46 2007
 *  Copyright  2007  Daniel Beck
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

#include <models/scanlines/star.h>

#include <utils/math/angle.h>

/** @class ScanlineStar <models/scanlines/star.h>
 * Star-like positioned scanline points.
 *
 * @author Daniel Beck
 */

/** Constructor.
 * @param image_width width of the image
 * @param image_height height of the image
 * @param center_x x-coordinate of the center point
 * @param center_y y-coordinate of the center point
 * @param num_segments number of segments
 * @param radius_incr number of pixels by which the radius is increased
 * @param dead_radius number of pixels around the center that are disregarded
 * @param max_radius maximal radius in number of pixels
 * @param margin margin around every scanline point that does not contain any
 *               other scanline point (in pixels)
 */
ScanlineStar::ScanlineStar( unsigned int image_width, unsigned int image_height,
			    unsigned int center_x, unsigned int center_y,
			    unsigned int num_segments, unsigned int radius_incr,
			    unsigned int dead_radius, unsigned int max_radius,
			    unsigned int margin )
{
  m_image_width = image_width;
  m_image_height = image_height;
  m_center_x = center_x;
  m_center_y = center_y;
  m_num_segments = num_segments;
  m_radius_incr = radius_incr;
  m_dead_radius = dead_radius;
  m_max_radius = max_radius;
  m_margin = margin;

  m_angle_incr = deg2rad( 360.0/m_num_segments );

  reset();
}


point_t
ScanlineStar::operator*()
{
  return m_current_coord;
}


point_t*
ScanlineStar::operator->()
{
  return &m_current_coord;
}


point_t*
ScanlineStar::operator++()
{
  advance();  
  return &m_current_coord;
}


point_t*
ScanlineStar::operator++(int)
{
  memcpy(&m_tmp_coord, &m_current_coord, sizeof(point_t));
  advance();

  return &m_tmp_coord;
}


/** Calculates the next scanline point. */
void
ScanlineStar::advance()
{
  bool ok = false;

  if (m_done) { return; }
  
  while (!ok)
    {
      if ( m_current_radius > m_max_radius )
	// check whether the maximal radius was exceeded
	{
	  m_current_angle += m_angle_incr;
	  m_current_radius = m_dead_radius;
	}

      if ( m_current_angle >= deg2rad(359.9) )
	// stop if we have looked at the whole 360°
	{
	  m_done = true;
	  ok = true;
	}
      else
	{
	  // calculate new (potential) scan point
	  point_t tmp;
	  tmp.x = m_center_x + (unsigned int)round(sin(m_current_angle) * m_current_radius);
	  tmp.y = m_center_y + (unsigned int)round(cos(m_current_angle) * m_current_radius);
	  
	  if (m_current_angle == 0)
	    // first ray; no values in last, yet.
	    {
	      m_current_coord = tmp;
	      m_first[m_current_radius] = m_current_coord;
	      m_last[m_current_radius] = m_current_coord;
	      ok = true;
	    }
	  else
	    {
	      // calculate distance to last approved point on that radius
	      float dist_first;
	      float dist_last;
	      int diff_x = tmp.x - m_first[m_current_radius].x;
	      int diff_y = tmp.y - m_first[m_current_radius].y;
	      dist_first = sqrt(diff_x * diff_x + diff_y * diff_y);
	      diff_x = tmp.x - m_last[m_current_radius].x;
	      diff_y = tmp.y - m_last[m_current_radius].y;
	      dist_last = sqrt(diff_x * diff_x + diff_y * diff_y);
	      
	      if (dist_first > 2 * m_margin && dist_last > 2 * m_margin)
		// approve point (and add it to last) if dist to last approved point
		// on the current radius is larger than twice the margin
		{
		  m_current_coord = tmp;
		  m_last[m_current_radius] = m_current_coord;
		  ok = true;
		}
	    }
	}
    
      m_current_radius += m_radius_incr;
    }
}


bool
ScanlineStar::finished()
{
  return m_done;
}


void
ScanlineStar::reset()
{
  m_done = false;
  
  m_current_radius = m_dead_radius;
  m_current_angle = 0.0f;

  m_current_coord.x = m_center_x + (unsigned int)floor(sin(m_current_angle) * m_current_radius);
  m_current_coord.y = m_center_y + (unsigned int)floor(cos(m_current_angle) * m_current_radius);

  // -- sanity checks --
  // max_radius
  unsigned int dist_x;
  unsigned int dist_y;
  unsigned int min;
  dist_x = (m_center_x > m_image_width / 2 ) ? (m_image_width - m_center_x) : m_center_x;
  dist_y = (m_center_y > m_image_height / 2 ) ? (m_image_height - m_center_y) : m_center_y;
  min = dist_x >= dist_y ? dist_y : dist_x;
  if (m_max_radius > min)
    {
      m_max_radius = min;
    }

  // margin
  if (m_margin > m_radius_incr / 2)
    {
      m_margin = m_radius_incr / 2;
    }
}


const char*
ScanlineStar::getName()
{
  return "ScanlineModel::Star";
}


unsigned int
ScanlineStar::getMargin()
{
  return m_margin;
}


void
ScanlineStar::setRobotPose(float x, float y, float ori)
{
  // ignored
}


void
ScanlineStar::setPanTilt(float pan, float tilt)
{
  // ignored
}


/** Skips the current ray and continues with the first scanline point of the
 * next ray. */
void
ScanlineStar::skip_current_ray()
{
  m_current_angle += m_angle_incr;
  m_current_radius = m_dead_radius;
}


/** Returns the number of segments in the model.
 * @return the number of segments
 */
unsigned int
ScanlineStar::num_segments() const
{
  return m_num_segments;
}


/** Returns the radius of the current scanline point.
 * @return the radius of the current scanline point
 */
unsigned int
ScanlineStar::current_radius() const
{
  return m_current_radius;
}


/** Returns the angle of the current scanline point
 * @return the angle of the current scanline point
 */
float
ScanlineStar::current_angle() const
{
  return m_current_angle;
}
