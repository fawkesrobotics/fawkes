
/***************************************************************************
 *  clips-webview-thread.cpp -  CLIPS introspection via webview
 *
 *  Created: Sat Jun 15 20:01:57 2013
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include "clips-webview-thread.h"
#include "clips-webview-processor.h"

#include <webview/url_manager.h>
#include <webview/nav_manager.h>

using namespace fawkes;

/** @class ClipsWebviewThread "clips-webview-thread.h"
 * Provide introspection for CLIPS via webview.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsWebviewThread::ClipsWebviewThread()
  : Thread("ClipsWebviewThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
ClipsWebviewThread::~ClipsWebviewThread()
{
}


void
ClipsWebviewThread::init()
{
  web_proc_  = new ClipsWebRequestProcessor(clips_env_mgr, logger);
  webview_url_manager->add_handler(WebRequest::METHOD_POST, "/clips/{env}/assert",
                                   std::bind(&ClipsWebRequestProcessor::process_assert, web_proc_,
                                             std::placeholders::_1));
  webview_url_manager->add_handler(WebRequest::METHOD_GET, "/clips/{env}/retract/{index}",
                                   std::bind(&ClipsWebRequestProcessor::process_retract, web_proc_,
                                             std::placeholders::_1));
  webview_url_manager->add_handler(WebRequest::METHOD_GET, "/clips/{env*}",
                                   std::bind(&ClipsWebRequestProcessor::process_environment, web_proc_,
                                             std::placeholders::_1));
  webview_url_manager->add_handler(WebRequest::METHOD_GET, "/clips",
                                   std::bind(&ClipsWebRequestProcessor::process_environment, web_proc_,
                                             std::placeholders::_1));
  webview_nav_manager->add_nav_entry("/clips/", "CLIPS");
}


void
ClipsWebviewThread::finalize()
{
	webview_url_manager->remove_handler(WebRequest::METHOD_POST, "/clips/{env}/assert");
	webview_url_manager->remove_handler(WebRequest::METHOD_GET, "/clips/{env}/retract/{index}");
	webview_url_manager->remove_handler(WebRequest::METHOD_GET, "/clips/{env*}");
	webview_url_manager->remove_handler(WebRequest::METHOD_GET, "/clips");
  webview_nav_manager->remove_nav_entry("/clips/");
  delete web_proc_;
}


void
ClipsWebviewThread::loop()
{
}
