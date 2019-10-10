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
#include "exog_manager.h"

#include <core/plugin.h>

using namespace fawkes;

/** @class GologppPlugin
 *  A plugin to integrate Golog++ into Fawkes.
 *  @author Victor Mataré
 */

class GologppPlugin : public Plugin
{
public:
	explicit GologppPlugin(Configuration *cfg);
	virtual ~GologppPlugin() override;
};

/** Constructor.
	 *  Create a thread that runs Golog++.
	 *  @param cfg The Fawkes configuration to be used by the plugin.
	 */
GologppPlugin::GologppPlugin(Configuration *cfg) : Plugin(cfg)
{
	fawkes_gpp::GologppThread *    exec_thread = new fawkes_gpp::GologppThread();
	fawkes_gpp::ExogManagerThread *exog_mgr    = new fawkes_gpp::ExogManagerThread(exec_thread);
	exec_thread->set_exog_mgr(exog_mgr);
	thread_list.push_back(exec_thread);
	thread_list.push_back(exog_mgr);
}

GologppPlugin::~GologppPlugin()
{
}

PLUGIN_DESCRIPTION("Golog++ Executive")
EXPORT_PLUGIN(GologppPlugin)
