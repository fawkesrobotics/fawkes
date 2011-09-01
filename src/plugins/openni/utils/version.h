
/***************************************************************************
 *  version.h - OpenNI utility methods: version comparison
 *
 *  Created: Tue Aug 30 15:44:56 2011
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

#ifndef __PLUGINS_OPENNI_UTILS_VERSION_H_
#define __PLUGINS_OPENNI_UTILS_VERSION_H_

#include <XnVersion.h>

namespace fawkes {
  namespace openni {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

#define XN_VERSION_GT(major, minor, micro, build)                       \
  (XN_VERSION > (major*100000000 + minor*1000000 + micro*10000 + build))

#define XN_VERSION_GE(major, minor, micro, build)                       \
  (XN_VERSION >= (major*100000000 + minor*1000000 + micro*10000 + build))

} // end namespace fawkes::openni
} // end namespace fawkes

#endif
