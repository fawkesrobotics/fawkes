
/***************************************************************************
 *  projection.h - Laser data projection filter
 *
 *  Created: Tue Mar 22 16:30:51 2011
 *  Copyright  2011  Christoph Schwering
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_LASER_FILTER_FILTERS_PROJECTION_H_
#define __PLUGINS_LASER_FILTER_FILTERS_PROJECTION_H_

#include "filter.h"

#include <vector>
#include <utility>
#include <string>

namespace fawkes {
  class Configuration;
  class Logger;
}

class LaserProjectionDataFilter : public LaserDataFilter
{
 public:
  struct Rotation {
    Rotation(float x_rot_degree, float y_rot_degree, float z_rot_degree)
        : x(x_rot_degree), y(y_rot_degree), z(z_rot_degree)
    { }

    float x;
    float y;
    float z;
  };

  struct Translation {
    Translation(float x_trans_degree, float y_trans_degree, float z_trans_degree)
        : x(x_trans_degree), y(y_trans_degree), z(z_trans_degree)
    { }

    float x; 
    float y;
    float z;
  };
  struct Rectangle {
    Rectangle(float x_min, float x_max, float y_min, float y_max)
        : x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max)
    { }

    float x_min;
    float x_max;
    float y_min;
    float y_max;
  };

  LaserProjectionDataFilter(const Rotation& laser_rot,
                            const Rotation& fixture_rot,
                            const Translation& trans,
                            const Rectangle& robot_rectangle,
                            float z_threshold,
                            unsigned int in_data_size,
                            std::vector<LaserDataFilter::Buffer *> &in);
  ~LaserProjectionDataFilter();

  void filter();

 private:
  inline void transform(const float angle, const float length,
                        float& new_angle, float& new_length,
                        bool& in_robot_rect, bool& too_low);

  const Rotation    LASER_ROT;
  const Rotation    FIXTURE_ROT;
  const Translation TRANS;
  const Rectangle   ROBOT;
  const float       Z_THRESHOLD;
};

#endif
