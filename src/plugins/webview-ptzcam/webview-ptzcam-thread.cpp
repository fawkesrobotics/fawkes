
/***************************************************************************
 *  webview-ptzcam-thread.cpp - Pan/tilt/zoom camera control for webview
 *
 *  Created: Fri Feb 07 16:02:11 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "webview-ptzcam-thread.h"
#include "webview-ptzcam-processor.h"

#include <webview/url_manager.h>
#include <webview/nav_manager.h>
#include <webview/request_manager.h>
#include <utils/time/time.h>
#include <utils/time/wait.h>

#include <interfaces/PanTiltInterface.h>
#include <interfaces/SwitchInterface.h>
#include <interfaces/CameraControlInterface.h>

#include <functional>

using namespace fawkes;

#define PTZCAM_URL_PREFIX "/ptzcam/"

/** @class WebviewPtzCamThread "webview-ptzcam-thread.h"
 * Pan/tilt/zoom camera control via webview.
 * @author Tim Niemueller
 */

/** Constructor. */
WebviewPtzCamThread::WebviewPtzCamThread()
  : Thread("WebviewPtzCamThread", Thread::OPMODE_CONTINUOUS)
{
  set_prepfin_conc_loop(true);
}


/** Destructor. */
WebviewPtzCamThread::~WebviewPtzCamThread()
{
}


void
WebviewPtzCamThread::init()
{
  timeout_ = false;

  std::string pantilt_id = config->get_string("/webview/ptzcam/pantilt-id");
  std::string camctrl_id = config->get_string("/webview/ptzcam/camctrl-id");
  std::string power_id   = config->get_string("/webview/ptzcam/power-id");
  std::string image_id   = config->get_string("/webview/ptzcam/image-id");
  std::string camera_id  = config->get_string("/webview/ptzcam/camera-id");

  float pan_increment         = config->get_float("/webview/ptzcam/pan-increment");
  float tilt_increment        = config->get_float("/webview/ptzcam/tilt-increment");
  unsigned int zoom_increment = config->get_uint("/webview/ptzcam/zoom-increment");
  float post_powerup_time     = config->get_float("/webview/ptzcam/post-power-up-time");

  std::string nav_entry = "PTZ Cam";
  try {
    nav_entry = config->get_string("/webview/ptzcam/nav-entry");
  } catch (Exception &e) {} // ignored, use default

  float loop_interval = config->get_float("/webview/ptzcam/loop-interval");
  long int loop_time = (long int)roundf(fabs(loop_interval) * 1000000.);

  cfg_inactivity_timeout_  = fabs(config->get_float("/webview/ptzcam/inactivity-timeout"));
  cfg_park_pan_tolerance_  = fabs(config->get_float("/webview/ptzcam/park/pan-tolerance"));
  cfg_park_pan_pos_        = fabs(config->get_float("/webview/ptzcam/park/pan"));
  cfg_park_tilt_tolerance_ = fabs(config->get_float("/webview/ptzcam/park/tilt-tolerance"));
  cfg_park_tilt_pos_       = fabs(config->get_float("/webview/ptzcam/park/tilt"));

  std::map<std::string, std::tuple<std::string, float, float, unsigned int>> presets;
  std::string prefix = "/webview/ptzcam/presets/";
#if __cplusplus >= 201103L
  std::unique_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
#else
  std::auto_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
#endif
  while (i->next()) {
    std::string cfg_name = std::string(i->path()).substr(prefix.length());
    cfg_name = cfg_name.substr(0, cfg_name.find("/"));

    if (presets.find(cfg_name) == presets.end()) {
      std::string cfg_prefix = prefix + cfg_name + "/";
      try {
	      std::string name  = config->get_string((cfg_prefix + "name").c_str());
	      float pan         = config->get_float((cfg_prefix + "pan").c_str());
	      float tilt        = config->get_float((cfg_prefix + "tilt").c_str());
	      unsigned int zoom = config->get_uint((cfg_prefix + "zoom").c_str());
	      presets[cfg_name] = std::make_tuple(name, pan, tilt, zoom);
      } catch (Exception &e) {
	      logger->log_warn(name(), "Invalid preset %s", cfg_name.c_str());
      }
    }
  }

  web_proc_  = new WebviewPtzCamRequestProcessor(image_id, pantilt_id, camctrl_id, power_id, camera_id,
                                                 pan_increment, tilt_increment,
                                                 zoom_increment, post_powerup_time,
                                                 presets,
                                                 blackboard, logger);

  webview_url_manager->add_handler(WebRequest::METHOD_GET, PTZCAM_URL_PREFIX "ping",
                                   std::bind(&WebviewPtzCamRequestProcessor::process_ping, web_proc_));
  webview_url_manager->add_handler(WebRequest::METHOD_GET, PTZCAM_URL_PREFIX "move",
                                   std::bind(&WebviewPtzCamRequestProcessor::process_move, web_proc_,
                                             std::placeholders::_1));
  webview_url_manager->add_handler(WebRequest::METHOD_GET, PTZCAM_URL_PREFIX "effect",
                                   std::bind(&WebviewPtzCamRequestProcessor::process_move, web_proc_,
                                             std::placeholders::_1));
  webview_url_manager->add_handler(WebRequest::METHOD_GET, PTZCAM_URL_PREFIX "?",
                                   std::bind(&WebviewPtzCamRequestProcessor::process_overview, web_proc_));

  webview_nav_manager->add_nav_entry(PTZCAM_URL_PREFIX, nav_entry.c_str());

  ptu_if_   = blackboard->open_for_reading<PanTiltInterface>(pantilt_id.c_str());
  power_if_ = blackboard->open_for_reading<SwitchInterface>(power_id.c_str());
  camen_if_ = blackboard->open_for_reading<SwitchInterface>(camera_id.c_str());

  bool ceiling_mount = false;
  try {
    ceiling_mount = config->get_bool("/webview/ptzcam/ceiling-mount");
  } catch (Exception &e) {} // ignored, use default

  if (ceiling_mount) {
    logger->log_info(name(), "Ceiling mode, ordering image mirroring");
    CameraControlInterface *camctrl_if =
	    blackboard->open_for_reading<CameraControlInterface>(camctrl_id.c_str());
    if (camctrl_if->has_writer()) {
	    CameraControlInterface::SetMirrorMessage *msg
		    = new CameraControlInterface::SetMirrorMessage(true);
	    camctrl_if->msgq_enqueue(msg);
    }
    blackboard->close(camctrl_if);
  }

  time_wait_ = new TimeWait(clock, loop_time);
}


void
WebviewPtzCamThread::finalize()
{
  webview_nav_manager->remove_nav_entry(PTZCAM_URL_PREFIX);
  webview_url_manager->remove_handler(WebRequest::METHOD_GET, PTZCAM_URL_PREFIX "ping");
  webview_url_manager->remove_handler(WebRequest::METHOD_GET, PTZCAM_URL_PREFIX "move");
  webview_url_manager->remove_handler(WebRequest::METHOD_GET, PTZCAM_URL_PREFIX "effect");
  webview_url_manager->remove_handler(WebRequest::METHOD_GET, PTZCAM_URL_PREFIX "?");
  delete web_proc_;

  blackboard->close(ptu_if_);
  blackboard->close(power_if_);
  blackboard->close(camen_if_);
  delete time_wait_;
}


void
WebviewPtzCamThread::loop()
{
  time_wait_->mark_start();

  if (webview_request_manager->num_active_requests() == 0) {
    fawkes::Time now(clock);
    try {
      Time last_completion =
	webview_request_manager->last_request_completion_time();

      if (now - &last_completion >= cfg_inactivity_timeout_) {
	if (! timeout_) {
	  logger->log_info(name(), "Inactivity timeout");
	  timeout_ = true;
	}
	ptu_if_->read();
	power_if_->read();
	camen_if_->read();
	if (fabs(cfg_park_pan_pos_  - ptu_if_->pan()) >= cfg_park_pan_tolerance_ ||
	    fabs(cfg_park_tilt_pos_ - ptu_if_->tilt()) >= cfg_park_tilt_tolerance_)
	{
	  PanTiltInterface::GotoMessage *gotomsg =
	    new PanTiltInterface::GotoMessage(cfg_park_pan_pos_, cfg_park_tilt_pos_);
	  ptu_if_->msgq_enqueue(gotomsg);
	} else {
	  if (power_if_->has_writer() && power_if_->is_enabled()) {
	    power_if_->msgq_enqueue(new SwitchInterface::DisableSwitchMessage());
	  }

	  if (camen_if_->has_writer() && camen_if_->is_enabled()) {
	    camen_if_->msgq_enqueue(new SwitchInterface::DisableSwitchMessage());
	  }
	}
      } else {
	timeout_ = false;
      }
    } catch (Exception &e) {} // ignore, a request got active
  }
  time_wait_->wait();
}
