
/***************************************************************************
 *  xabsl_plugin.h - Fawkes XABSL Plugin
 *
 *  Created: Wed Aug 06 16:51:28 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <plugins/xabsl/xabsl_plugin.h>

#include "engine_thread.h"

using namespace fawkes;

/** @class XabslPlugin <plugins/xabsl/xabsl_plugin.h>
 * XABSL plugin for Fawkes.
 * This plugin integrates XABSL into Fawkes. It is meant to serve as an
 * example and is not a fully supported control plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
XabslPlugin::XabslPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new XabslEngineThread());
}


PLUGIN_DESCRIPTION("Integrates XABSL interpreter")
EXPORT_PLUGIN(XabslPlugin)
