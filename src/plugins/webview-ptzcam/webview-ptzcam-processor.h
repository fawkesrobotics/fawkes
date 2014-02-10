
/***************************************************************************
 *  webview-ptzcam-processor.h - Pan/tilt/zoom camera control for webview
 *
 *  Created: Fri Feb 07 17:51:06 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WEBVIEW_PTZCAM_WEBVIEW_PTZCAM_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_PTZCAM_WEBVIEW_PTZCAM_PROCESSOR_H_

#include <webview/request_processor.h>

#include <string>

namespace fawkes {
  class Logger;
  class BlackBoard;
  class PanTiltInterface;
  class CameraControlInterface;
  class SwitchInterface;
}

class WebviewPtzCamRequestProcessor : public fawkes::WebRequestProcessor
{
 public:
  WebviewPtzCamRequestProcessor(std::string base_url, std::string image_id, std::string pantilt_id,
				std::string camctrl_id, std::string power_id,
				float pan_increment, float tilt_increment, unsigned int zoom_increment,
				float post_powerup_time,
				fawkes::BlackBoard *blackboard, fawkes::Logger *logger);

  virtual ~WebviewPtzCamRequestProcessor();

  virtual fawkes::WebReply * process_request(const fawkes::WebRequest *request);

 private:

 private:
  fawkes::Logger       *logger_;
  fawkes::BlackBoard   *blackboard_;

  fawkes::PanTiltInterface       *ptu_if_;
  fawkes::CameraControlInterface *camctrl_if_;
  fawkes::SwitchInterface        *power_if_;

  std::string           baseurl_;
  std::string           image_id_;

  float                 pan_increment_;
  float                 tilt_increment_;
  long int              zoom_increment_;
  long int              post_powerup_time_;
};

#endif
