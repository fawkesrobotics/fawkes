
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

#define CLIPS_URL_PREFIX "/clips"

/** @class ClipsWebviewThread "clips-webview-thread.h"
 * Provide introspection for CLIPS via webview.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param env_name CLIPS environment name to which to register
 */
ClipsWebviewThread::ClipsWebviewThread(std::string &env_name)
  : Thread("ClipsWebviewThread", Thread::OPMODE_WAITFORWAKEUP),
    CLIPSAspect(env_name.c_str(), /* create */ true, /* excl */ false)
{
}


/** Destructor. */
ClipsWebviewThread::~ClipsWebviewThread()
{
}


void
ClipsWebviewThread::init()
{
/*
  cfg_proto_dirs_.clear();
  try {
    cfg_proto_dirs_ = config->get_strings("/clips-webview/proto-dirs");
    for (size_t i = 0; i < cfg_proto_dirs_.size(); ++i) {
      std::string::size_type pos;
      if ((pos = cfg_proto_dirs_[i].find("@BASEDIR@")) != std::string::npos) {
	cfg_proto_dirs_[i].replace(pos, 9, BASEDIR);
      }
      if ((pos = cfg_proto_dirs_[i].find("@FAWKES_BASEDIR@")) != std::string::npos) {
	cfg_proto_dirs_[i].replace(pos, 16, FAWKES_BASEDIR);
      }
      if ((pos = cfg_proto_dirs_[i].find("@RESDIR@")) != std::string::npos) {
	cfg_proto_dirs_[i].replace(pos, 8, RESDIR);
      }
      if ((pos = cfg_proto_dirs_[i].find("@CONFDIR@")) != std::string::npos) {
	cfg_proto_dirs_[i].replace(pos, 9, CONFDIR);
      }
      if (cfg_proto_dirs_[i][cfg_proto_dirs_.size()-1] != '/') {
	cfg_proto_dirs_[i] += "/";
      }
      //logger->log_warn(name(), "DIR: %s", cfg_proto_dirs_[i].c_str());
    }
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to load proto paths from config, exception follows");
    logger->log_warn(name(), e);
  } // ignore, use default
*/
  web_proc_  = new ClipsWebRequestProcessor(clips, logger, CLIPS_URL_PREFIX);
  webview_url_manager->register_baseurl(CLIPS_URL_PREFIX, web_proc_);
  webview_nav_manager->add_nav_entry(CLIPS_URL_PREFIX, "CLIPS");
}


void
ClipsWebviewThread::finalize()
{
  webview_url_manager->unregister_baseurl(CLIPS_URL_PREFIX);
  webview_nav_manager->remove_nav_entry(CLIPS_URL_PREFIX);
  delete web_proc_;
}


void
ClipsWebviewThread::loop()
{
}
