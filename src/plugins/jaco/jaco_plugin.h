
/***************************************************************************
 *  jaco_plugin.h - Fawkes Kinova Jaco Plugin
 *
 *  Created: Tue Jun 04 13:13:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_JACO_JACO_PLUGIN_H_
#define __PLUGINS_JACO_JACO_PLUGIN_H_

#include <core/plugin.h>

class JacoPlugin : public fawkes::Plugin
{
 public:
  JacoPlugin(fawkes::Configuration *config);
};

#endif
