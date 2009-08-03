
/***************************************************************************
 *  dummy_service_publisher.h - publish services
 *
 *  Created: Fri Jun 29 15:30:17 2007 (on flight to Atlanta, RoboCup 2007)
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

#ifndef __NETCOMM_SERVICE_DISCOVERY_DUMMY_SERVICE_PUBLISHER_H_
#define __NETCOMM_SERVICE_DISCOVERY_DUMMY_SERVICE_PUBLISHER_H_

#include <netcomm/service_discovery/service_publisher.h>

namespace fawkes {

class DummyServicePublisher : public ServicePublisher
{
 public:
  DummyServicePublisher();
  virtual ~DummyServicePublisher();

  virtual void publish_service(NetworkService *service);
  virtual void unpublish_service(NetworkService *service);
};

} // end namespace fawkes

#endif
