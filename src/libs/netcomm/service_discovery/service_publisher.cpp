
/***************************************************************************
 *  service_publisher.cpp - publish services
 *
 *  Created: Fri Jun 29 14:35:11 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <netcomm/service_discovery/service_publisher.h>

/** @class ServicePublisher <netcomm/service_discovery/service_publisher.h>
 * Service publisher interface.
 *
 * @fn void ServicePublisher::publish(NetworkService *service) = 0
 * Publish service.
 * @param service service to publish
 */

/** Virtual empty destructor. */
ServicePublisher::~ServicePublisher()
{
}
