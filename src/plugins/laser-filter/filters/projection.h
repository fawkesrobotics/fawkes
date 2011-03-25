
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
  LaserProjectionDataFilter(bool left,
                            float x_rot, float y_rot, float z_rot,
                            float x_trans, float y_trans, float z_trans,
                            unsigned int in_data_size,
                            std::vector<float *> in);
  ~LaserProjectionDataFilter();

  void filter();

 private:
  inline void transform(const float angle, const float length,
                        float& new_angle, float& new_length);

  const bool LEFT;

  const float X_ROT;
  const float Y_ROT;
  const float Z_ROT;

  const float X_TRANS;
  const float Y_TRANS;
  const float Z_TRANS;

};

#endif
