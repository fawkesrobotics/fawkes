
/***************************************************************************
 *  url_manager.cpp - Web URL manager
 *
 *  Created: Thu Nov 25 21:56:19 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include <webview/url_manager.h>
#include <webview/router.h>
#include <core/threading/mutex.h>
#include <core/exception.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class WebUrlManager <webview/url_manager.h>
 * Manage URL mappings.
 * This class maps (base) URLs to web request processors which handle all
 * requests for the given URL.
 * @author Tim Niemueller
 */

/** Constructor. */
WebUrlManager::WebUrlManager()
	: router_(std::make_shared<WebviewRouter<Handler>>())
{
}


/** Destructor. */
WebUrlManager::~WebUrlManager()
{
}


/** Add a request processor.
 * @param method HTTP method to register for
 * @param path path pattern to register for, may contain {var}, {var*}, and {var+} elements
 * @param handler handler function
 * @exception Exception thrown if a processor has already been registered
 * for the given URL prefix.
 */
void
WebUrlManager::add_handler(WebRequest::Method method, const std::string& path, Handler handler)
{
	std::lock_guard<std::mutex> lock(mutex_);
	router_->add(method, path, handler, 0);
}

/** Add a request processor with weight.
 * This one should mostly be necessary to implement "catch-all" handlers.
 * @param method HTTP method to register for
 * @param path path pattern to register for, may contain {var}, {var*}, and {var+} elements
 * @param handler handler function
 * @param weight the higher the weight the later the handler will be tried.
 * @exception Exception thrown if a processor has already been registered
 * for the given URL prefix.
 */
void
WebUrlManager::add_handler(WebRequest::Method method, const std::string& path, Handler handler, int weight)
{
	std::lock_guard<std::mutex> lock(mutex_);
	router_->add(method, path, handler, weight);
}


/** Remove a request processor.
 * @param method HTTP method to unregister from
 * @param path path pattern to unregister from
 */
void
WebUrlManager::remove_handler(WebRequest::Method method, const std::string& path)
{
	std::lock_guard<std::mutex> lock(mutex_);
  router_->remove(method, path);
}

/** Lock mutex and find processor.
 * This method determines if a processor has been registered for the URL.
 * It is the callers duty to ensure that the mutex has been locked while
 * searching and while using the found processor.
 * @param url url to get the processor for
 * @return request processor if found, NULL otherwise
 */
WebReply *
WebUrlManager::process_request(WebRequest *request)
{
	std::lock_guard<std::mutex> lock(mutex_);
	try {
		std::map<std::string, std::string> path_args;
		Handler handler = router_->find_handler(request, path_args);
		request->set_path_args(std::move(path_args));
		return handler(request);
	} catch (NullPointerException &e) {
		return NULL;
	}
}

} // end namespace fawkes
