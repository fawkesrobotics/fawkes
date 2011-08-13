
/***************************************************************************
 *  dcm_utils.cpp - DCM utility functions
 *
 *  Created: Thu Aug 11 11:00:26 2011
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

#include "dcm_utils.h"

namespace dcm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

void
set_value(AL::ALPtr<AL::DCMProxy> &dcm,
          const std::string &device, const std::string &kind,
          float value, int time)
{
  AL::ALValue cmd, actcmd, actpos;

  cmd.arrayPush(device); cmd.arrayPush(kind);
  actpos.arrayPush(value); actpos.arrayPush(time);
  actcmd.arrayPush(actpos); cmd.arrayPush(actcmd);

  dcm->set(cmd);
}


std::vector<std::string>
get_devices(AL::ALPtr<AL::DCMProxy> &dcm, AL::ALPtr<AL::ALMemoryProxy> &almem,
            std::string type)
{
  AL::ALValue names = almem->getDataListName();
  std::string subd_prefix = dcm->getPrefix()[0];

  std::vector<std::string> rv;

  // Walk sub-device tree and extract joints
  for (unsigned int i = 0; i < names.getSize(); ++i) {
    std::string name = names[i];
    if ( name.compare(0, subd_prefix.length(), subd_prefix) == 0 ) {
      if ( name.compare(name.length() - 5, 5, "/Type") == 0 ) {
	std::string dtype = almem->getData(name, 0);
	std::string base_path = name.substr(0, name.length() - 5);
	if ( dtype == type ) {
          rv.push_back(base_path);
        }
      }
    }
  }

  return rv;
}

} // end of namespace dcm
