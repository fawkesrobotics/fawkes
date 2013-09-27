
/***************************************************************************
 *  star.h - Starlike scanline model
 *
 *  Created: Mon Nov 05 09:45:06 2007
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

#ifndef __FIREVISION_MODELS_SCANLINES_STAR_H_
#define __FIREVISION_MODELS_SCANLINES_STAR_H_

#include <fvmodels/scanlines/scanlinemodel.h>
#include <vector>
#include <map>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ScanlineStar : public ScanlineModel
{
 public:
  ScanlineStar(unsigned int image_width, unsigned int image_height,
         unsigned int center_x, unsigned int center_y,
         unsigned int num_rays, unsigned int radius_incr,
         unsigned char* yuv_mask,
         unsigned int dead_radius = 0, unsigned int max_radius = 0,
         unsigned int margin = 0);

  virtual ~ScanlineStar();

  fawkes::upoint_t    operator*();
  fawkes::upoint_t *  operator->();
  fawkes::upoint_t *  operator++();
  fawkes::upoint_t *  operator++(int);

  void advance();
  bool finished();
  void reset();
  const char* get_name();
  unsigned int get_margin();
  void set_robot_pose(float x, float y, float ori);
  void set_pan_tilt(float pan, float tilt);
  void skip_current_ray();
  unsigned int num_rays() const;
  unsigned int ray_index() const;
  unsigned int current_radius() const;
  float current_angle() const;
  bool first_on_ray() const;

 private:
  void generate_scan_points();

  unsigned int m_image_width;
  unsigned int m_image_height;
  fawkes::upoint_t m_center;
  unsigned int m_num_rays;
  unsigned int m_radius_incr;
  unsigned int m_dead_radius;
  unsigned int m_max_radius;
  unsigned int m_margin;
  float m_angle_incr;
  unsigned char* m_mask;

  bool m_first_on_ray;
  bool m_done;

  fawkes::upoint_t m_current_point;
  fawkes::upoint_t m_tmp_point;
  unsigned int m_ray_index;

  typedef std::map<unsigned int, fawkes::upoint_t> Ray;
  std::map<float, Ray*> m_rays;
  std::map<float, Ray*>::iterator m_ray_iter;
  Ray::iterator m_point_iter;

  //  std::vector<float> m_angles;
  //  std::vector<float>::iterator m_angle_iter;

  Ray* m_first_ray;
  Ray* m_previous_ray;
};

} // end namespace firevision

#endif /* __FIREVISION_MODELS_SCANLINES_STAR_H_ */
