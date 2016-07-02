
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

#ifndef HAVE_TF
#  error LaserProjectionDataFilter only availabe with TF
#endif

#include <tf/transformer.h>

#include <string>

namespace fawkes {
  class Configuration;
  class Logger;
}

class LaserProjectionDataFilter : public LaserDataFilter
{
 public:
	LaserProjectionDataFilter(std::string filter_name,
                            fawkes::tf::Transformer *tf,
                            std::string target_frame,
                            float not_from_x, float not_to_x,
                            float not_from_y, float not_to_y,
                            float only_from_z, float only_to_z,
                            unsigned int in_data_size,
                            std::vector<LaserDataFilter::Buffer *> &in);
  ~LaserProjectionDataFilter();

  void filter();

 private:
  inline void set_output(float *outbuf, fawkes::tf::Point &p);

 private:
  fawkes::tf::Transformer *tf_;
  const std::string target_frame_;
  const float not_from_x_, not_to_x_;
  const float not_from_y_, not_to_y_;
  const float only_from_z_, only_to_z_;

  float sin_angles360[360];
  float cos_angles360[360];
  float sin_angles720[720];
  float cos_angles720[720];

  float index_factor_;
};

#endif
