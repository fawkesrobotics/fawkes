
/***************************************************************************
 *  filter.h - Laser data filter interface
 *
 *  Created: Fri Oct 10 17:11:04 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_FILTER_H_
#define __PLUGINS_LASER_FILTER_H_

class LaserDataFilter
{
 public:
  LaserDataFilter();
  virtual ~LaserDataFilter();

  virtual float *       filtered_data();
  virtual unsigned int  filtered_data_size();
  virtual void          filtered_data(float *&data, unsigned int &data_size);
  virtual void          filter(const float *data, unsigned int data_size)   = 0;

 protected:
  float        *_filtered_data;
  unsigned int  _filtered_data_size;
  bool          _free_filtered_data;
};


#endif
