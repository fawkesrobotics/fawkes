
/***************************************************************************
 *  backendinfo-rest-api.h -  Backend Info REST API
 *
 *  Created: Mon Apr 09 15:42:54 2018
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
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/webview.h>

#include <webview/rest_api.h>
#include <webview/rest_array.h>

#include "model/Backend.h"

class BackendInfoRestApi
: public fawkes::Thread,
	public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
	public fawkes::WebviewAspect
{
 public:
	BackendInfoRestApi();
	~BackendInfoRestApi();

	virtual void init();
	virtual void loop();
	virtual void finalize();

 private:
	WebviewRestArray<Backend> cb_list_backends();

 private:
	fawkes::WebviewRestApi        *rest_api_;
	WebviewRestArray<Backend>      backends_;
};
