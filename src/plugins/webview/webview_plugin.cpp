
/***************************************************************************
 *  webview_plugin.h - Fawkes Webview Plugin
 *
 *  Created: Mon Oct 13 17:46:57 2008 (I5 Developer's Day)
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include <plugins/webview/webview_plugin.h>
#include <core/exception.h>

#include "webview_thread.h"

#ifdef HAVE_REST_APIS
#  include "blackboard-rest-api/blackboard-rest-api.h"
#  include "backendinfo-rest-api/backendinfo-rest-api.h"
#  include "plugin-rest-api/plugin-rest-api.h"
#  include "config-rest-api/config-rest-api.h"
#  ifdef HAVE_JPEG
#    include "image-rest-api/image-rest-api.h"
#  endif
#  ifdef HAVE_TF
#    include "tf-rest-api/tf-rest-api.h"
#  endif
#endif

using namespace fawkes;

/** @class WebviewPlugin <plugins/webview/webview_plugin.h>
 * Webview plugin for Fawkes.
 * This provides an extensible web interface for Fawkes.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
WebviewPlugin::WebviewPlugin(Configuration *config)
  : Plugin(config)
{
	bool enable_thread_pool =
		config->get_bool("/webview/thread-pool/enable");

  thread_list.push_back(new WebviewThread(enable_thread_pool));
#ifdef HAVE_REST_APIS
  thread_list.push_back(new BlackboardRestApi());
  thread_list.push_back(new BackendInfoRestApi());
  thread_list.push_back(new PluginRestApi());
  thread_list.push_back(new ConfigurationRestApi());
#  ifdef HAVE_JPEG
  thread_list.push_back(new ImageRestApi());
#  endif
#  ifdef HAVE_TF
  thread_list.push_back(new TransformsRestApi());
#  endif
#endif
}


PLUGIN_DESCRIPTION("Web interface for Fawkes")
EXPORT_PLUGIN(WebviewPlugin)
