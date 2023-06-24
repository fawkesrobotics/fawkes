
/***************************************************************************
 *  amcl_plugin.cpp - Adaptive Monte Carlo Localization plugin
 *
 *  Created: Wed May 16 16:02:15 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include "amcl2_thread.h"
#include "amcl_utils.h"
#include "ros2_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Adaptive Monte Carlo Localization plugin.
 * @author Tim Niemueller
 */
class Amcl2Plugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit Amcl2Plugin(Configuration *config) : Plugin(config)
	{
		AmclROS2Thread *rt          = NULL;
		bool            ros_enabled = true;
		try {
			ros_enabled = config->get_bool(AMCL_CFG_PREFIX "ros/enable");
		} catch (Exception &e) {
		} // ignore, use default
		if (ros_enabled) {
			rt = new AmclROS2Thread();
			thread_list.push_back(rt);
		}
		thread_list.push_back(new Amcl2Thread(rt));
	}
};

PLUGIN_DESCRIPTION("Adaptive Monte Carlo Localization")
EXPORT_PLUGIN(Amcl2Plugin)
