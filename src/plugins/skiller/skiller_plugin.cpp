
/***************************************************************************
 *  skiller_plugin.h - Fawkes Skill Execution Runtime Plugin
 *
 *  Created: Mon Feb 18 10:18:50 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include "skiller_plugin.h"
#include "exec_thread.h"

using namespace fawkes;

/** @class SkillerPlugin <plugins/skiller/skiller_plugin.h>
 * Skill Execution Runtime Plugin.
 * This plugin facilitates the Fawkes skill module. It allows for execution of
 * so-called skills, basic action atoms that can be used for form complex
 * behavior.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
SkillerPlugin::SkillerPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new SkillerExecutionThread());
}


PLUGIN_DESCRIPTION("Behavior engine based on Lua")
EXPORT_PLUGIN(SkillerPlugin)
