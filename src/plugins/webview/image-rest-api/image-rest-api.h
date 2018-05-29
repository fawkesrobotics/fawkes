
/***************************************************************************
 *  blackboard-rest-api.h -  Blackboard REST API
 *
 *  Created: Mon Mar 26 23:26:40 2018
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
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/webview.h>
#include <aspect/blackboard.h>
#include <aspect/thread_producer.h>

#include <webview/rest_api.h>
#include <webview/rest_array.h>

#include "model/ImageInfo.h"

#include <map>
#include <string>
#include <utility>

namespace fawkes {
  class WebviewJpegStreamProducer;
}

class ImageRestApi
: public fawkes::Thread,
	public fawkes::ClockAspect,
	public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
	public fawkes::BlackBoardAspect,
  public fawkes::ThreadProducerAspect,
	public fawkes::WebviewAspect
{
 public:
	ImageRestApi();
	~ImageRestApi();

	virtual void init();
	virtual void loop();
	virtual void finalize();

 private:
	WebviewRestArray<ImageInfo> cb_list_images();

	std::shared_ptr<fawkes::WebviewJpegStreamProducer>
		get_stream(const std::string& image_id);

	std::unique_ptr<fawkes::WebReply> cb_get_image(fawkes::WebviewRestParams& params);

 private:
	fawkes::WebviewRestApi        *rest_api_;
	std::map<std::string, std::shared_ptr<fawkes::WebviewJpegStreamProducer>> streams_;
};
