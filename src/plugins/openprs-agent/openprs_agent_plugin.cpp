
/***************************************************************************
 *  openprs_agent_plugin.cpp - OpenPRS agent plugin
 *
 *  Created: Fri Aug 22 13:55:28 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#include "openprs_agent_thread.h"
#include <core/plugin.h>
#include <config/config.h>

using namespace fawkes;

/** Agent executive using OpenPRS.
 * @author Tim Niemueller
 */
class OpenPRSAgentPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  OpenPRSAgentPlugin(Configuration *config) : Plugin(config)
  {
    OpenPRSAspect::Mode mode = OpenPRSAspect::OPRS;
    bool gdb_delay = false;
    try {
      std::string mode_s = config->get_string("/openprs-agent/oprs-mode");
      if (mode_s == "XOPRS")  mode = OpenPRSAspect::XOPRS;
    } catch (Exception &e) {} // ignored, use default
    try {
      gdb_delay = config->get_bool("/openprs-agent/gdb-delay");
    } catch (Exception &e) {} // ignored, use default
    thread_list.push_back(new OpenPRSAgentThread(mode, gdb_delay));
  }
};


PLUGIN_DESCRIPTION("OpenPRS agent executive")
EXPORT_PLUGIN(OpenPRSAgentPlugin)
