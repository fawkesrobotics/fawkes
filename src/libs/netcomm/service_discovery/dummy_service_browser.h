
/***************************************************************************
 *  dummy_service_browser.h - browse services
 *
 *  Created: Fri Jun 29 15:24:15 2007 (on the flight to RoboCup 2007, Atlanta)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __NETCOMM_SERVICE_DISCOVERY_DUMMY_SERVICE_BROWSER_H_
#define __NETCOMM_SERVICE_DISCOVERY_DUMMY_SERVICE_BROWSER_H_

#include <netcomm/service_discovery/service_browser.h>

namespace fawkes {

class DummyServiceBrowser : public ServiceBrowser
{
 public:
  DummyServiceBrowser();
  virtual ~DummyServiceBrowser();

  virtual void watch_service(const char *service_type, ServiceBrowseHandler *h);
  virtual void unwatch_service(const char *service_type, ServiceBrowseHandler *h);
};

} // end namespace fawkes


#endif
