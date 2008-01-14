
/***************************************************************************
 *  star.h - Starlike scanline model
 *
 *  Generated: Mon Nov 05 09:45:06 2007
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

#ifndef __FIREVISION_MODELS_SCANLINES_STAR_H_
#define __FIREVISION_MODELS_SCANLINES_STAR_H_

#include <models/scanlines/scanlinemodel.h>
#include <vector>
#include <map>

class ScanlineStar : public ScanlineModel
{
 public:
  ScanlineStar(unsigned int image_width, unsigned int image_height,
	       unsigned int center_x, unsigned int center_y,
	       unsigned int num_segments, unsigned int radius_incr,
	       unsigned char* yuv_mask,
	       unsigned int dead_radius = 0, unsigned int max_radius = 0,
	       unsigned int margin = 0);

  virtual ~ScanlineStar();
    
  point_t operator*();
  point_t* operator->();
  point_t* operator++();
  point_t* operator++(int);

  void advance();
  bool finished();
  void reset();
  const char* getName();
  unsigned int getMargin();
  void setRobotPose(float x, float y, float ori);
  void setPanTilt(float pan, float tilt);
  void skip_current_ray();
  unsigned int num_segments() const;
  unsigned int current_radius() const;
  float current_angle() const;

 private:
  void generate_scan_points();
  //  point_t& get_point

  unsigned int m_image_width;
  unsigned int m_image_height;
  point_t m_center;
  unsigned int m_num_segments;
  unsigned int m_radius_incr;
  unsigned int m_dead_radius;
  unsigned int m_max_radius;
  unsigned int m_margin;
  float m_angle_incr;
  unsigned char* m_mask;

  bool m_done;

  point_t m_current_point;
  point_t m_tmp_point;

  typedef std::map<unsigned int, point_t> Ray;
  std::map<float, Ray*> m_rays;
  std::vector<float> m_angles;
  std::vector<float>::iterator m_angle_iter;
  std::vector<unsigned int> m_radii;
  std::vector<unsigned int>::iterator m_radius_iter;

  Ray m_first_ray;
  Ray m_previous_ray;
};

#endif /* __FIREVISION_MODELS_SCANLINES_STAR_H_ */
