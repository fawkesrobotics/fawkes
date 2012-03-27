
/***************************************************************************
 *  srsave_plugin.h - SwissRanger Save Plugin
 *
 *  Created: Fri Jan 22 10:48:38 2010
 *  Copyright  2007-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_SRSAVE_PLUGIN_H_
#define __FIREVISION_APPS_SRSAVE_PLUGIN_H_

#include <core/plugin.h>

class FvSrSavePlugin : public fawkes::Plugin
{
 public:
  FvSrSavePlugin(fawkes::Configuration *config);
};

#endif
