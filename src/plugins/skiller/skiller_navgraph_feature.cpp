/***************************************************************************
 *  skiller_navgraph_thread.cpp - Optional skiller access to navgraph
 *
 *  Created: Wed Jul 16 13:03:25 2014 (on flight to Joao Pessoa)
 *  Copyright  2012-2014  Tim Niemueller [www.niemueller.de]
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

#include "skiller_navgraph_feature.h"

#include <navgraph/yaml_navgraph.h>
#include <lua/context.h>

using namespace fawkes;

/** @class SkillerNavGraphFeature "skiller_navgraph_feature.h"
 * Thread to access the navgraph from skiller.
 * The thread itself does not perform any actions. But splitting it into its
 * own thread will allow us to make access to the navgraph optional. So if
 * the graph is not required the navgraph module does not have to be loaded.
 * @author Tim Niemueller
 */

/** Constructor. */
SkillerNavGraphFeature::SkillerNavGraphFeature()
  : Thread("SkillerNavGraphFeature", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
SkillerNavGraphFeature::~SkillerNavGraphFeature()
{
}

void
SkillerNavGraphFeature::init()
{
}

void
SkillerNavGraphFeature::finalize()
{
}

void
SkillerNavGraphFeature::loop()
{
}


void
SkillerNavGraphFeature::init_lua_context(fawkes::LuaContext *context)
{
  logger->log_info(name(), "Intializing navgraph for skiller");
  context->add_package("fawkesnavgraph");
  context->get_global("features_env_template");
  context->push_string("navgraph");
  context->push_usertype(*navgraph, "NavGraph", "fawkes");
  context->set_table();
}

void
SkillerNavGraphFeature::finalize_lua_context(fawkes::LuaContext *context)
{
  logger->log_info(name(), "Finalizing navgraph for skiller");
  context->get_global("features_env_template");
  context->push_string("navgraph");
  context->push_nil();
  context->set_table();
}
