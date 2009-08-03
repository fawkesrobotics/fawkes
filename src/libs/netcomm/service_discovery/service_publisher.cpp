
/***************************************************************************
 *  service_publisher.cpp - publish services
 *
 *  Created: Fri Jun 29 14:35:11 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/service_discovery/service_publisher.h>

namespace fawkes {

/** @class ServicePublisher <netcomm/service_discovery/service_publisher.h>
 * Service publisher interface.
 *
 * @fn void ServicePublisher::publish_service(NetworkService *service) = 0
 * Publish service.
 * @param service service to publish
 *
 * @fn void ServicePublisher::unpublish_service(NetworkService *service) = 0
 * Revoke service publication.
 * @param service service to revoke
 */

/** Virtual empty destructor. */
ServicePublisher::~ServicePublisher()
{
}

} // end namespace fawkes
