
/***************************************************************************
 *  setup.h - OpenNI utility methods: setup routines
 *
 *  Created: Thu Mar 24 10:21:31 2011
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

#ifndef __PLUGINS_OPENNI_UTILS_SETUP_H_
#define __PLUGINS_OPENNI_UTILS_SETUP_H_

#include <XnCppWrapper.h>
#include <string>

namespace fawkes {
  class Configuration;

  namespace openni {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

void setup_map_generator(xn::MapGenerator &generator,
			 fawkes::Configuration *config);

} // end namespace fawkes::openni
} // end namespace fawkes

#endif
