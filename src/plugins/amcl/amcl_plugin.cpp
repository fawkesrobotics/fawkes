
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

#include "amcl_thread.h"

#ifndef HAVE_ROS
# error "ROS integration missing"
#endif


#include <core/plugin.h>
#ifdef HAVE_ROS
#	include "amcl_utils.h"
#	include "ros_thread.h"
#endif

using namespace fawkes;

/** Adaptive Monte Carlo Localization plugin.
 * @author Tim Niemueller
 */
class AmclPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit AmclPlugin(Configuration *config) : Plugin(config)
	{
#ifdef HAVE_ROS
		AmclROS2Thread *rt          = NULL;
		bool           ros_enabled = true;
		try {
			ros_enabled = config->get_bool(AMCL_CFG_PREFIX "ros/enable");
		} catch (Exception &e) {
		} // ignore, use default
		if (ros_enabled) {
			rt = new AmclROS2Thread();
			thread_list.push_back(rt);
		}
		thread_list.push_back(new AmclThread(rt));
#else
		thread_list.push_back(new AmclThread());
#endif
	}
};

PLUGIN_DESCRIPTION("Adaptive Monte Carlo Localization")
EXPORT_PLUGIN(AmclPlugin)
