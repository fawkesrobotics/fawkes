
/***************************************************************************
 *  tf_processor.h - Web request processor for TF data
 *
 *  Created: Wed Jun 19 17:44:36 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WEBVIEW_TF_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_TF_PROCESSOR_H_

#include <tf/transformer.h>

#include <map>
#include <string>

namespace fawkes {
  class BlackBoard;
  class Interface;
  class WebUrlManager;
  class WebReply;
  class WebRequest;
}

class WebviewTfRequestProcessor
{
 public:
	WebviewTfRequestProcessor(fawkes::WebUrlManager *url_manager,
                            fawkes::tf::Transformer *transformer);
  ~WebviewTfRequestProcessor();

 private:
  fawkes::WebReply * process_graph();
  fawkes::WebReply * process_overview();

 private:
  fawkes::tf::Transformer *transformer_;
  fawkes::WebUrlManager *url_manager_;

};

#endif
