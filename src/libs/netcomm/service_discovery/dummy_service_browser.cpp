
/***************************************************************************
 *  dummy_service_browser.cpp - browse services
 *
 *  Created: Fri Jun 29 15:26:10 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/service_discovery/dummy_service_browser.h>

/** @class Dummy ServiceBrowser <netcomm/service_discovery/service_browser.h>
 * Dummy Service browser.
 * Does nothing, used to fulfill NetworkAspect guarantees if Avahi is not available.
 */


/** Constructor. */
DummyServiceBrowser::DummyServiceBrowser()
{
}


/** Virtual empty destructor. */
DummyServiceBrowser::~DummyServiceBrowser()
{
}


void
DummyServiceBrowser::add_handler(const char *service_type, ServiceBrowseHandler *h)
{
}


void
DummyServiceBrowser::remove_handler(const char *service_type, ServiceBrowseHandler *h)
{
}

