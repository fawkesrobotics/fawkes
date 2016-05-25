
/***************************************************************************
 *  syncpoint_processor.h - Web request processor for SyncPoints
 *
 *  Created: Thu Aug 14 16:21:42 2014
 *  Copyright  2014  Till Hofmann
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

#ifndef __PLUGINS_WEBVIEW_SYNCPOINT_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_SYNCPOINT_PROCESSOR_H_

#include <webview/request_processor.h>

#include <core/utils/refptr.h>
#include <syncpoint/syncpoint_manager.h>

namespace fawkes {
  class BlackBoard;
  class Interface;
}

class WebviewSyncPointRequestProcessor : public fawkes::WebRequestProcessor
{
 public:
  WebviewSyncPointRequestProcessor(const char *baseurl,
    fawkes::SyncPointManager *syncpoint_manager, float max_age);
  virtual ~WebviewSyncPointRequestProcessor();

  virtual fawkes::WebReply * process_request(const fawkes::WebRequest *request);

 protected:
  std::string all_syncpoints_as_dot(
    const std::set< fawkes::RefPtr<fawkes::SyncPoint>,
                    fawkes::SyncPointSetLessThan > syncpoints,
    float max_age);

 private:
  char *baseurl_;
  size_t baseurl_len_;
  fawkes::SyncPointManager *syncpoint_manager_;
  float max_age_;

};

#endif
