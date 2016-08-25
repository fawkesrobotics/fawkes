
/***************************************************************************
 *  cascade.h - Laser data filter cascade
 *
 *  Created: Thu Jun 25 01:04:59 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_FILTER_FILTERS_CASCADE_H_
#define __PLUGINS_LASER_FILTER_FILTERS_CASCADE_H_

#include "filter.h"

#include <list>

class LaserDataFilterCascade : public LaserDataFilter
{
 public:
  LaserDataFilterCascade(const std::string filter_name,
                         unsigned int in_data_size, std::vector<Buffer *> &in);
  ~LaserDataFilterCascade();

  virtual void set_out_vector(std::vector<LaserDataFilter::Buffer *> &out);

  void add_filter(LaserDataFilter *filter);
  void remove_filter(LaserDataFilter *filter);
  void delete_filters();

  /** Check if filters have been added to the cascade.
   * @return true if filters have been registered, false otherwise */
  inline bool has_filters() const { return ! __filters.empty(); }

  void filter();

  /** Get filters.
   * @return list of active filters. */
  const std::list<LaserDataFilter *> &  get_filters() const { return __filters; }

 private:
  std::list<LaserDataFilter *>           __filters;
  std::list<LaserDataFilter *>::iterator __fit;
};


#endif
