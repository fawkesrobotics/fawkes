
/***************************************************************************
 *  compatibility.h - Helper definitions for PCL's boost -> std transition
 *
 *  Copyright  2020  Victor Mataré
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

#pragma once

#include <pcl/pcl_config.h>

#if PCL_VERSION_COMPARE(<, 1, 11, 0)

#	include <boost/smart_ptr/shared_ptr.hpp>

namespace pcl {
template <class T>
using shared_ptr = boost::shared_ptr<T>;
}

#else

#	include <pcl/memory.h>

#endif
