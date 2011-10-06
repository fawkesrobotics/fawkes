
/***************************************************************************
 *  dcm_utils.h - DCM utility functions
 *
 *  Created: Thu Aug 11 10:57:50 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAO_DCM_UTILS_H_
#define __PLUGINS_NAO_DCM_UTILS_H_

#include <alcore/alptr.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>

#include <vector>
#include <string>

namespace dcm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

void
set_value(AL::ALPtr<AL::DCMProxy> &dcm,
          const std::string &device, const std::string &kind,
          float value, int time);

std::vector<std::string>
get_devices(AL::ALPtr<AL::DCMProxy> &dcm, AL::ALPtr<AL::ALMemoryProxy> &almem,
            std::string type);

} // end of namespace dcm

#endif
