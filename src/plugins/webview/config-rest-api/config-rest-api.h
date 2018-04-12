
/***************************************************************************
 *  config-rest-api.h - Configuration REST API
 *
 *  Created: Thu Apr 12 18:59:38 2018
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
#include <aspect/configurable.h>

#include <webview/rest_api.h>
#include <webview/rest_array.h>

#include "model/ConfigTree.h"

#include <map>
#include <string>

class ConfigurationRestApi
: public fawkes::Thread,
  public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::WebviewAspect
{
 public:
	ConfigurationRestApi();
	~ConfigurationRestApi();

	virtual void init();
	virtual void loop();
	virtual void finalize();

 private:
	ConfigTree cb_get_config(fawkes::WebviewRestParams &params);

 private:
	fawkes::WebviewRestApi        *rest_api_;
};
