
/***************************************************************************
 *  plugins_processor.h - Web request processor for plugin info
 *
 *  Created: Thu Feb 12 12:59:25 2009
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

#ifndef __PLUGINS_WEBVIEW_PLUGINS_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_PLUGINS_PROCESSOR_H_

namespace fawkes {
  class PluginManager;
  class WebReply;
  class WebRequest;
  class WebUrlManager;
}

class WebviewPluginsRequestProcessor
{
 public:
	WebviewPluginsRequestProcessor(fawkes::WebUrlManager *url_manager,
	                               fawkes::PluginManager *manager);
  ~WebviewPluginsRequestProcessor();

 private:
  fawkes::WebReply * process_load(const fawkes::WebRequest *request);
  fawkes::WebReply * process_unload(const fawkes::WebRequest *request);
  fawkes::WebReply * process_overview();

 private:
  fawkes::PluginManager *manager_;
  fawkes::WebUrlManager *url_manager_;
};

#endif
