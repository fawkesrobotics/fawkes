/***************************************************************************
 *  plugin.cpp - Golog++ plugin
 *
 *  Created: Mon 26 Aug 2019 CEST 15:38
 *  Copyright  2019  Victor Mataré <matare@fh-aachen.de>
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
#include "execution_thread.h"

#include <core/plugin.h>

using namespace fawkes;

class GologppPlugin : public Plugin
{
public:
	explicit GologppPlugin(Configuration *cfg) : Plugin(cfg)
	{
		thread_list.push_back(new GologppThread());
	}
};

PLUGIN_DESCRIPTION("Detect the conveyor belt in a pointcloud")
EXPORT_PLUGIN(GologppPlugin)
