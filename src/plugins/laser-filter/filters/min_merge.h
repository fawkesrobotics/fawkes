
/***************************************************************************
 *  min_merge.h - Laser min merge data filter
 *
 *  Created: Wed Mar 16 21:45:27 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_FILTER_FILTERS_MIN_MERGE_H_
#define __PLUGINS_LASER_FILTER_FILTERS_MIN_MERGE_H_

#include "filter.h"

#include <vector>

namespace fawkes {
	class Logger;
}

class LaserMinMergeDataFilter : public LaserDataFilter
{
 public:
  /// Timestamp selection method
  typedef enum {
    TIMESTAMP_LATEST,	///< use the latest of all timestamps
    TIMESTAMP_FIRST,	///< use the first (oldest) of all timestamps
    TIMESTAMP_INDEX	///< use a specific index in the input buffer list
  } TimestampSelectionMethod;

  LaserMinMergeDataFilter(const std::string filter_name,
                          fawkes::Logger *logger,
                          unsigned int in_data_size,
                          std::vector<LaserDataFilter::Buffer *> &in);
  LaserMinMergeDataFilter(const std::string filter_name,
                          fawkes::Logger *logger,
                          unsigned int in_data_size,
                          std::vector<LaserDataFilter::Buffer *> &in,
                          TimestampSelectionMethod timestamp_selection_method,
                          unsigned int timestamp_index = 0);

  virtual void filter();

 private:
  fawkes::Logger *logger;

  TimestampSelectionMethod timestamp_selection_method_;
  unsigned int timestamp_index_;

  std::vector<bool> ignored_;
};

#endif
