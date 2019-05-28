
/***************************************************************************
 *  realsense2_plugin.cpp - realsense2
 *
 *  Plugin created: Wed May 22 10:09:22 2018

 *  Copyright  2019 Christoph Gollok
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

#include "realsense2_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Driver for the Intel RealSense2 cameras.
 * @author Christoph Gollok
 */
class Realsense2Plugin : public fawkes::Plugin
{
public:
	/** Constructor
   * @param config Fakwes configuration
   */
	explicit Realsense2Plugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new Realsense2Thread());
	}
};

PLUGIN_DESCRIPTION("Driver for Intel RealSense2 camera")
EXPORT_PLUGIN(Realsense2Plugin)
