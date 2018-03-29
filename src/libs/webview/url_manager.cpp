
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
#include <webview/request_processor.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
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
{
  mutex_ = new Mutex();
}


/** Destructor. */
WebUrlManager::~WebUrlManager()
{
  delete mutex_;
}


/** Add a request processor.
 * @param url_prefix baseurl this processor should handle
 * @param processor processor for baseurl
 * @exception Exception thrown if a processor has already been registered
 * for the given URL prefix.
 */
void
WebUrlManager::register_baseurl(const char *url_prefix,
                                WebRequestProcessor *processor)
{
  MutexLocker lock(mutex_);
  if (processors_.find(url_prefix) != processors_.end()) {
	  throw Exception("A processor for %s has already been registered",
	                  url_prefix);
  }
  processors_[url_prefix] = processor;
}


/** Remove a request processor.
 * @param url_prefix baseurl the processor handled
 */
void
WebUrlManager::unregister_baseurl(const char *url_prefix)
{
  MutexLocker lock(mutex_);
  processors_.erase(url_prefix);
}

/** Lock mutex and find processor.
 * This method determines if a processor has been registered for the URL.
 * It is the callers duty to ensure that the mutex has been locked while
 * searching and while using the found processor.
 * @param url url to get the processor for
 * @return request processor if found, NULL otherwise
 */
WebRequestProcessor *
WebUrlManager::find_processor(const std::string &url) const
{
  auto proc = std::find_if(processors_.begin(), processors_.end(),
                           [&url](const auto &p) {
	                           return url.find(p.first) == 0;
                           });
  if (proc != processors_.end()) {
	  return proc->second;
  } else {
	  return NULL;
  }
}


/** Get internal mutex.
 * Use this mutex to guard find_processor() and a following invocation of
 * a found processor against changes due to registering/unregistering of
 * processors.
 * @return internal mutex
 */
Mutex *
WebUrlManager::mutex()
{
  return mutex_;
}

} // end namespace fawkes
