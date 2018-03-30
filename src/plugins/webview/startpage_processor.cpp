
/***************************************************************************
 *  startpage_processor.cpp - Web request processor for the start page
 *
 *  Created: Thu Feb 12 00:10:53 2009
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

#include "startpage_processor.h"
#include <webview/page_reply.h>
#include <webview/url_manager.h>

#include <logging/cache.h>

#include <functional>

using namespace fawkes;

/** @class WebviewStartPageRequestProcessor "startpage_processor.h"
 * Web request processor for the start page.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param url_manager URL manager to register with
 * @param cache_logger cache logger
 */
WebviewStartPageRequestProcessor::WebviewStartPageRequestProcessor(fawkes::WebUrlManager *url_manager,
                                                                   CacheLogger *cache_logger)
{
  cache_logger_ = cache_logger;
  url_manager_ = url_manager;
  url_manager_->add_handler(WebRequest::METHOD_GET, "/",
                            std::bind(&WebviewStartPageRequestProcessor::process_request, this));
}


/** Destructor. */
WebviewStartPageRequestProcessor::~WebviewStartPageRequestProcessor()
{
	url_manager_->remove_handler(WebRequest::METHOD_GET, "/");
}


WebReply *
WebviewStartPageRequestProcessor::process_request()
{
	WebPageReply *r = new WebPageReply("Fawkes", "<h1>Welcome to Fawkes.</h1>\n");

	std::list<CacheLogger::CacheEntry> & messages = cache_logger_->get_messages();
	std::list<CacheLogger::CacheEntry>::reverse_iterator i;

	*r += "<h2>Latest log messages</h2>\n";
	*r += "<table>\n";
	for (i = messages.rbegin(); i != messages.rend(); ++i) {
		CacheLogger::CacheEntry &e = *i;
		const char *color = NULL;
		switch (e.log_level) {
		case Logger::LL_DEBUG: color = "#888888"; break;
		case Logger::LL_WARN:  color = "orange";  break;
		case Logger::LL_ERROR: color = "red";     break;
		default: ;
		}
		if (color) {
			r->append_body("<tr><td>%s</td><td>%s</td><td><span style=\"color:%s\">%s</span></td></tr>\n",
			               e.timestr.c_str(), e.component.c_str(), color, e.message.c_str());
		} else {
			r->append_body("<tr><td>%s</td><td>%s</td><td>%s</td></tr>\n",
			               e.timestr.c_str(), e.component.c_str(), e.message.c_str());
		}
	}
	*r += "</table>\n";

	return r;
}
