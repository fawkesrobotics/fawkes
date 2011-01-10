
/***************************************************************************
 *  rrd_example_plugin.h - Fawkes RRD Example Plugin
 *
 *  Created: Mon Jan 10 00:05:30 2011
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_RRD_EXAMPLE_RRD_EXAMPLE_PLUGIN_H_
#define __PLUGINS_RRD_EXAMPLE_RRD_EXAMPLE_PLUGIN_H_

#include <core/plugin.h>

class RRDExamplePlugin : public fawkes::Plugin
{
 public:
  RRDExamplePlugin(fawkes::Configuration *config);
};

#endif
