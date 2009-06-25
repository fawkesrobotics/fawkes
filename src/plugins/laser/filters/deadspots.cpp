
/***************************************************************************
 *  deadspots.cpp - Laser data dead spots filter
 *
 *  Created: Wed Jun 24 22:42:51 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include "deadspots.h"

#include <core/exception.h>
#include <core/macros.h>
#include <utils/math/angle.h>
#include <utils/logging/logger.h>
#include <config/config.h>

#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <regex.h>

using namespace fawkes;

/** @class LaserDeadSpotsDataFilter "filters/deadspots.h"
 * Erase dead spots (i.e. mounting rods in the laser range) from laser data.
 * This filter reads a number of values stored in /hardware/laser/deadspots, where
 * each dead spot must contain two entries, a start and an end in degrees. Each
 * entry is stored as submembers of the given tree, for example as
 * /hardware/laser/deadspots/0/start and /hardware/laser/deadspots/0/end.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config configuration instance
 * @param logger logger for informational output
 * @param prefix configuration prefix where to log for config information
 */
LaserDeadSpotsDataFilter::LaserDeadSpotsDataFilter(fawkes::Configuration *config,
						   fawkes::Logger *logger,
						   std::string prefix)
{
  __logger = logger;

  regex_t pathre;
  int error = 0;
  if ((error = regcomp(&pathre, (prefix + "\\([^/]\\+\\)/\\(start\\|end\\)").c_str(), 0)) != 0) {
    size_t errsize = regerror(error, &pathre, NULL, 0);
    char tmp[errsize];
    regerror(error, &pathre, tmp, errsize);
    regfree(&pathre);
    throw Exception("Failed to compile regular expression: %s", tmp);
  }

  regmatch_t matches[2];

  std::list<std::string> entries;

  Configuration::ValueIterator *vit = config->search(prefix.c_str());
  while (vit->next()) {
    const char *path = vit->path();
    if (regexec(&pathre, path, 2, matches, 0) == 0) {
      unsigned int match1_length = matches[1].rm_eo - matches[1].rm_so;

      char entry[match1_length + 1];    entry[match1_length] = 0;
      strncpy(entry, &(path[matches[1].rm_so]), match1_length);
      entries.push_back(entry);
    }
  }
  delete vit;
  entries.sort();
  entries.unique();

  __dead_spots = new unsigned int[entries.size() * 2];

  for (std::list<std::string>::iterator i = entries.begin(); i != entries.end(); ++i) {
    std::string path = prefix + *i + "/";
    float start = config->get_float((path + "start").c_str()); 
    float end   = config->get_float((path + "end").c_str()); 

    __logger->log_debug("LaserDeadSpotsDataFilter", "Adding dead range [%3.3f, %3.3f] (%s)",
			start, end, i->c_str());
    __cfg_dead_spots.push_back(std::make_pair(start, end));
  }

  __num_spots = __cfg_dead_spots.size();

  if (__num_spots == 0) {
    throw Exception("Dead spots filter enabled but no calibration data exists. Run fflaser_deadspots.");
  }
}

LaserDeadSpotsDataFilter::~LaserDeadSpotsDataFilter()
{
  delete __dead_spots;
}


void
LaserDeadSpotsDataFilter::filter(const float *data, unsigned int data_size)
{
  if (unlikely(_filtered_data_size != data_size)) {
    // need to calculate new beam ranges and allocate different memory segment
    float angle_factor = 360.0 / data_size;
    for (unsigned int i = 0; i < __num_spots; ++i) {
      __dead_spots[i * 2    ] = std::min(data_size - 1, (unsigned int)ceilf(__cfg_dead_spots[i].first  / angle_factor));
      __dead_spots[i * 2 + 1] = std::min(data_size - 1, (unsigned int)ceilf(__cfg_dead_spots[i].second / angle_factor));
    }
    if (_filtered_data)  free(_filtered_data);
    _filtered_data      = (float *)malloc(sizeof(float) * data_size);
    _filtered_data_size = data_size;
  }

  unsigned int start = 0;
  for (unsigned int i = 0; i < __num_spots; ++i) {
    const unsigned int spot_start = __dead_spots[i * 2    ];
    const unsigned int spot_end   = __dead_spots[i * 2 + 1];
    for (unsigned int j = start; j < spot_start; ++j) {
      _filtered_data[j] = data[j];
    }
    for (unsigned int j = spot_start; j <= spot_end; ++j) {
      _filtered_data[j] = 0.0;
    }
    start = spot_end + 1;
  }
  for (unsigned int j = start; j < data_size; ++j) {
    _filtered_data[j] = data[j];
  }
}
