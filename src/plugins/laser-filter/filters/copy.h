/***************************************************************************
 *  copy.h - Laser data filter to copy data without modification
 *
 *  Created: Mon 16 Apr 2018 13:50:26 CEST 13:50
 *  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#ifndef __PLUGINS_LASER_FILTER_FILTERS_COPY_H_
#define __PLUGINS_LASER_FILTER_FILTERS_COPY_H_

#include "filter.h"

class LaserCopyDataFilter : public LaserDataFilter
{
  public:
    LaserCopyDataFilter(const std::string filter_name, uint in_data_size,
        std::vector<Buffer *> &in);
    void filter();
};

#endif /* !__PLUGINS_LASER_FILTER_FILTERS_COPY_H_ */
