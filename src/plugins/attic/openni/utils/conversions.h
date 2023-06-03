
/***************************************************************************
 *  conversions.h - OpenNI utility methods: conversions
 *
 *  Created: Thu Mar 31 21:22:19 2011
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

#ifndef _PLUGINS_OPENNI_UTILS_CONVERSIONS_H_
#define _PLUGINS_OPENNI_UTILS_CONVERSIONS_H_

#include <core/exception.h>
#include <core/utils/lockptr.h>

#include <XnCppWrapper.h>
#include <string>

namespace fawkes {

namespace openni {

void world2projection(xn::DepthGenerator *depthgen,
                      unsigned int        num_points,
                      const XnPoint3D    *world,
                      XnPoint3D          *proj,
                      unsigned int        width  = 0,
                      unsigned int        height = 0);

} // namespace openni
} // end namespace fawkes

#endif
