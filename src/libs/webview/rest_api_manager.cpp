
/***************************************************************************
 *  rest_api_manager.cpp - Web REST API manager
 *
 *  Created: Fri Mar 16 16:49:02 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include <webview/rest_api_manager.h>
#include <webview/rest_api.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/exception.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class WebviewRestApiManager <webview/url_manager.h>
 * Manage URL mappings.
 * This class maps (base) URLs to web request processors which handle all
 * requests for the given URL.
 * @author Tim Niemueller
 */

/** Constructor. */
WebviewRestApiManager::WebviewRestApiManager()
{
}


/** Destructor. */
WebviewRestApiManager::~WebviewRestApiManager()
{
}


/** Add a REST API.
 * @param api REST api handler
 * @exception Exception thrown if an API of that name has already been
 * registered
 */
void
WebviewRestApiManager::register_api(WebviewRestApi *api)
{
  MutexLocker lock(&mutex_);
  if (apis_.find(api->name()) != apis_.end()) {
	  throw Exception("A REST API for %s has already been registered",
	                  api->name().c_str());
  }
  apis_[api->name()] = api;
}


/** Remove a request processor.
 * @param api REST api handler
 */
void
WebviewRestApiManager::unregister_api(WebviewRestApi *api)
{
  MutexLocker lock(&mutex_);
  apis_.erase(api->name());
}

/** Find API by name.
 * This method determines if a processor has been registered for the URL.
 * It is the callers duty to ensure that the mutex has been locked while
 * searching and while using the found processor.
 * @param name name of REST API to retrieve
 * @return request processor if found, NULL otherwise
 */
WebviewRestApi *
WebviewRestApiManager::get_api(std::string &name)
{
	if (apis_.find(name) == apis_.end()) {
		return NULL;
	}
	return apis_[name];
}


/** Get internal mutex.
 * Use this mutex to guard find_processor() and a following invocation of
 * a found processor against changes due to registering/unregistering of
 * processors.
 * @return internal mutex
 */
Mutex &
WebviewRestApiManager::mutex()
{
  return mutex_;
}

} // end namespace fawkes
