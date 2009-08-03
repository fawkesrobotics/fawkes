
/***************************************************************************
 *  service_browser.cpp - browse services
 *
 *  Created: Fri Jun 29 14:39:00 2007 (on flight to RoboCup 2007, Atlanta)
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

#include <netcomm/service_discovery/service_browser.h>

namespace fawkes {

/** @class ServiceBrowser <netcomm/service_discovery/service_browser.h>
 * Service browser.
 *
 * @fn void ServiceBrowser::watch_service(const char *service_type, ServiceBrowseHandler *h) = 0
 * Add browse handler for specific service.
 * @param service_type type of service to browse for, implementation dependant.
 * @param h browse handler to add for this service.
 *
 * @fn void ServiceBrowser::unwatch_service(const char *service_type, ServiceBrowseHandler *h) = 0
 * Remove browse handler for specific service.
 * @param service_type type of service to browse for, implementation dependant.
 * @param h browse handler to remove for this service.
 */

/** Virtual empty destructor. */
ServiceBrowser::~ServiceBrowser()
{
}

} // end namespace fawkes
