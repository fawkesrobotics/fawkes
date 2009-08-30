
/***************************************************************************
 *  request_processor.h - Web request processor
 *
 *  Created: Mon Oct 13 21:58:49 2008
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

#ifndef __PLUGINS_WEBVIEW_REQUEST_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_REQUEST_PROCESSOR_H_

#include "reply.h"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class WebRequestProcessor
{
 public:
  virtual ~WebRequestProcessor();
  virtual WebReply * process_request(const char *url,
				     const char *method,
				     const char *version,
				     const char *upload_data,
				     size_t *upload_data_size,
				     void **session_data)               = 0;

};

} // end namespace fawkes

#endif
