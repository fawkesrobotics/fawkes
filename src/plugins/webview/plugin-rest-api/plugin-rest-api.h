
/***************************************************************************
 *  plugin-rest-api.h -  Plugin REST API
 *
 *  Created: Tue Apr 10 17:08:41 2018
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

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/webview.h>
#include <aspect/plugin_director.h>

#include <webview/rest_api.h>
#include <webview/rest_array.h>

#include "model/Plugin.h"
#include "model/PluginOpRequest.h"
#include "model/PluginOpResponse.h"

#include <map>
#include <string>

class PluginRestApi
: public fawkes::Thread,
  public fawkes::LoggingAspect,
	public fawkes::PluginDirectorAspect,
	public fawkes::WebviewAspect
{
 public:
	PluginRestApi();
	~PluginRestApi();

	virtual void init();
	virtual void loop();
	virtual void finalize();

 private:
	WebviewRestArray<::Plugin> cb_list_plugins();
	PluginOpResponse cb_set_plugin_state(PluginOpRequest request,
	                                     fawkes::WebviewRestParams &params);

 private:
	fawkes::WebviewRestApi        *rest_api_;
};
