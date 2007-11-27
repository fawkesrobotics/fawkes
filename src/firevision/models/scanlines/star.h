
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
#include <map>

class ScanlineStar : public ScanlineModel
{
 public:
  ScanlineStar(unsigned int image_widht, unsigned int image_height,
	       unsigned int center_x, unsigned int center_y,
	       unsigned int num_segments, unsigned int radius_incr,
	       unsigned int dead_radius = 0, unsigned int max_radius = 0,
	       unsigned int margin = 0);
  
  
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
  unsigned int m_image_width;
  unsigned int m_image_height;
  unsigned int m_center_x;
  unsigned int m_center_y;
  unsigned int m_num_segments;
  unsigned int m_radius_incr;
  unsigned int m_dead_radius;
  unsigned int m_max_radius;
  unsigned int m_margin;
  float m_angle_incr;

  bool m_done;

  unsigned int m_current_radius;
  float m_current_angle;
  point_t m_current_coord;
  point_t m_tmp_coord;

  std::map<unsigned int, point_t> m_first;
  std::map<unsigned int, point_t> m_last;
};

#endif /* __FIREVISION_MODELS_SCANLINES_STAR_H_ */
