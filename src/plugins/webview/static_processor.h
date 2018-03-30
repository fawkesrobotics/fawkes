
/***************************************************************************
 *  static_processor.h - Web request processor for static files
 *
 *  Created: Mon Oct 13 23:41:34 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WEBVIEW_STATIC_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_STATIC_PROCESSOR_H_

#include <vector>
#include <string>
#include <map>

namespace fawkes {
  class Logger;
  class WebUrlManager;
  class WebReply;
  class WebRequest;
}

class WebviewStaticRequestProcessor
{
 public:
	WebviewStaticRequestProcessor(fawkes::WebUrlManager *url_manager,
	                              const std::string& base_url,
	                              std::vector<std::string>& htdocs_dir,
	                              const std::string& catchall_file,
	                              const std::string& mime_file,
	                              fawkes::Logger *logger);
  ~WebviewStaticRequestProcessor();

 private:
  fawkes::WebReply * process_request(const fawkes::WebRequest *request);
  std::string find_file(const std::string& filename);
  void read_mime_database(const std::string& mime_file);
  const std::string & get_mime_type(const std::string& file_name);

 private:
  std::vector<std::string> htdocs_dirs_;

  fawkes::Logger *logger_;
  fawkes::WebUrlManager *url_manager_;

  std::map<std::string, std::string> mime_types_;

  std::string base_url_;
  std::string catchall_file_;

};

#endif
