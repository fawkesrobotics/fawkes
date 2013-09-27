
/***************************************************************************
 *  star.cpp - Starlike scanline model
 *
 *  Created: Mon Nov 05 10:06:46 2007
 *  Copyright  2007  Daniel Beck
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

#include <fvmodels/scanlines/star.h>
#include <fvutils/color/yuv.h>
#include <utils/math/angle.h>

#include <cstring>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ScanlineStar <fvmodels/scanlines/star.h>
 * Star-like arranged scanline points.
 *
 * @author Daniel Beck
 */

/** Constructor.
 * @param image_width width of the image
 * @param image_height height of the image
 * @param center_x x-coordinate of the center point
 * @param center_y y-coordinate of the center point
 * @param num_rays number of rays
 * @param radius_incr number of pixels by which the radius is increased
 * @param yuv_mask a mask allows to exclude certain regions of the image from
 *        inspection. More precisely, no scanline points are generated in those
 *        areas. The ignored regions have to be black, i.e. Y=0, U=127, V=127.
 * @param dead_radius number of pixels around the center that are disregarded
 * @param max_radius maximal radius in number of pixels
 * @param margin margin around every scanline point that does not contain any
 *               other scanline point (in pixels)
 */
ScanlineStar::ScanlineStar( unsigned int image_width, unsigned int image_height,
                            unsigned int center_x, unsigned int center_y,
                            unsigned int num_rays, unsigned int radius_incr,
                            unsigned char* yuv_mask,
                            unsigned int dead_radius, unsigned int max_radius,
                            unsigned int margin)
{
  m_image_width = image_width;
  m_image_height = image_height;
  m_center.x = center_x;
  m_center.y = center_y;
  m_num_rays = num_rays;
  m_radius_incr = radius_incr;
  m_mask = yuv_mask;
  m_dead_radius = dead_radius;
  m_max_radius = max_radius;
  m_margin = margin;

  m_angle_incr = deg2rad( 360.0/m_num_rays );

  m_first_ray = 0;
  m_previous_ray = 0;

  m_first_on_ray = true;

  // -- sanity checks --
  // margin
  if (m_margin > m_radius_incr / 2)
    {
      m_margin = m_radius_incr / 2;
    }

  generate_scan_points();

  reset();
}


/** Destructor. */
ScanlineStar::~ScanlineStar()
{
  std::map<float, Ray*>::iterator rit;
  for (rit = m_rays.begin(); rit != m_rays.end(); ++rit)
    {
      delete rit->second;
    }
}

upoint_t
ScanlineStar::operator*()
{
  return m_current_point;
}


upoint_t*
ScanlineStar::operator->()
{
  return &m_current_point;
}


upoint_t*
ScanlineStar::operator++()
{
  advance();
  return &m_current_point;
}


upoint_t*
ScanlineStar::operator++(int)
{
  memcpy(&m_tmp_point, &m_current_point, sizeof(upoint_t));
  advance();

  return &m_tmp_point;
}


/** Calculates the next scanline point. */
void
ScanlineStar::advance()
{
  if (m_done) { return; }

  ++m_point_iter;
  m_first_on_ray = false;

  if ( (*m_ray_iter).second->end() == m_point_iter )
    {
      ++m_ray_iter;

      if ( m_rays.end() == m_ray_iter )
        {
          m_done = true;
          return;
        }

      ++m_ray_index;
      m_point_iter = (*m_ray_iter).second->begin();
      m_first_on_ray = true;
    }

  m_current_point = (*m_point_iter).second;
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
  m_first_on_ray = true;

  m_ray_index = 0;
  m_ray_iter = m_rays.begin();
  m_point_iter = (*m_ray_iter).second->begin();
  m_current_point = (*m_point_iter).second;
}


const char*
ScanlineStar::get_name()
{
  return "ScanlineModel::Star";
}


unsigned int
ScanlineStar::get_margin()
{
  return m_margin;
}


void
ScanlineStar::set_robot_pose(float x, float y, float ori)
{
  // ignored
}


void
ScanlineStar::set_pan_tilt(float pan, float tilt)
{
  // ignored
}


/** Skips the current ray and continues with the first valid scanline point of
 * the next ray. */
void
ScanlineStar::skip_current_ray()
{
  if (m_done) { return; }

  ++m_ray_iter;

  if ( m_rays.end() == m_ray_iter )
    {
      m_done = true;
      return;
    }

  ++m_ray_index;
  m_first_on_ray = true;
  m_point_iter = m_ray_iter->second->begin();
  m_current_point = (*m_point_iter).second;
}


/** Returns the number of segments in the model.
 * @return the number of segments
 */
unsigned int
ScanlineStar::num_rays() const
{
  return m_num_rays;
}


/** Return the index of the current ray.
 * @return the index of the current ray
 */
unsigned int
ScanlineStar::ray_index() const
{
  return m_ray_index;
}


/** Returns the radius of the current scanline point.
 * @return the radius of the current scanline point
 */
unsigned int
ScanlineStar::current_radius() const
{
  return m_point_iter->first;
}


/** Returns the angle of the current scanline point
 * @return the angle of the current scanline point
 */
float
ScanlineStar::current_angle() const
{
  return m_ray_iter->first;
}

/** Checks whether the current scanpoint is the first scanpoint on the
 * current ray.
 * @return true, if the it is the first scanpoint on the current ray
 */
bool
ScanlineStar::first_on_ray() const
{
  return m_first_on_ray;
}

void
ScanlineStar::generate_scan_points()
{
  float angle = 0.0;
  unsigned int radius;
  Ray* current_ray;
  bool abort_ray;
  YUV_t ignore(0);

  while (angle < deg2rad(359.9) )
    {
      abort_ray = false;
      radius = m_dead_radius;
      current_ray = new Ray();

      while ( !abort_ray )
        {
          // calculate new (potential) scan point
          upoint_t tmp;
          tmp.x = m_center.x + (unsigned int) round( sin(angle) * radius );
          tmp.y = m_center.y + (unsigned int) round( cos(angle) * radius );

          YUV_t current;
          if ( tmp.x >= m_image_width || tmp.y >= m_image_height )
            // outside of the image
            {
              current = ignore;
              abort_ray = true;
            }
          else
            // get mask value
            {
              current.Y = YUV422_PLANAR_Y_AT(m_mask, m_image_width, tmp.x, tmp.y);
              current.U = YUV422_PLANAR_U_AT(m_mask, m_image_width, m_image_height, tmp.x, tmp.y);
              current.V = YUV422_PLANAR_V_AT(m_mask, m_image_width, m_image_height, tmp.x, tmp.y);
            }

          if ( ignore.Y != current.Y &&
               ignore.U != current.U &&
               ignore.V != current.V )
            // not masked
            {
              if (0 == m_previous_ray)
                // no previous values, yet.
                {
                  (*current_ray)[radius] = tmp;
                  m_first_ray = current_ray;
                }
              else
                {
                  // calculate distance to last approved point on that radius
                  float dist_first = 3 * m_margin;
                  float dist_last = 3 * m_margin;
                  int diff_x;
                  int diff_y;

                  if ( m_first_ray->find(radius) != m_first_ray->end() )
                    {
                      diff_x = tmp.x - (*m_first_ray)[radius].x;
                      diff_y = tmp.y - (*m_first_ray)[radius].y;
                      dist_first = sqrt(diff_x * diff_x + diff_y * diff_y);
                    }
                  if ( m_previous_ray->find(radius) != m_previous_ray->end() )
                    {
                      diff_x = tmp.x - (*m_previous_ray)[radius].x;
                      diff_y = tmp.y - (*m_previous_ray)[radius].y;
                      dist_last = sqrt(diff_x * diff_x + diff_y * diff_y);
                    }

                  if (dist_first > 2 * m_margin && dist_last > 2 * m_margin)
                    // approve point (and add it to previous) if dist to last approved point
                    // on the current radius is larger than twice the margin
                    {
                      (*current_ray)[radius] = tmp;
                    }
                }
            }

          radius += m_radius_incr;

          if (radius > m_max_radius) { abort_ray = true; }
        }

      if ( !current_ray->empty() )
        // there are scanpoints on this ray
        {
          m_rays[angle] = current_ray;
          m_previous_ray = current_ray;
        }
      else
        {
          delete current_ray;
        }

      angle += m_angle_incr;
    }

  m_num_rays = m_rays.size();

  /*
  unsigned int num_rays = m_rays.size();
  unsigned int num_points = 0;

  std::map<float, Ray*>::iterator rit;
  for (rit = m_rays.begin(); rit != m_rays.end(); ++rit)
    {
      num_points += (*rit).second->size();
    }
  printf("Generated %d points in %d rays\n", num_points, num_rays);
  */
}

} // end namespace firevision
