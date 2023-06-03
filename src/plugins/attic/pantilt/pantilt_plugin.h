
/***************************************************************************
 *  pantilt_plugin.h - Plugin to drive pan/tilt units (e.g. for cameras)
 *
 *  Created: Wed Jun 17 19:26:24 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_PANTILT_PANTILT_PLUGIN_H_
#define _PLUGINS_PANTILT_PANTILT_PLUGIN_H_

#include <core/plugin.h>

class PanTiltPlugin : public fawkes::Plugin
{
public:
	explicit PanTiltPlugin(fawkes::Configuration *config);
};

#endif
