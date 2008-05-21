
/***************************************************************************
 *  navigator_plugin.cpp - Navigator Plugin
 *
 *  Generated: Thu May 31 18:13:16 2007
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
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

#include <plugins/navigator/navigator_plugin.h>
#include <plugins/navigator/navigator_thread.h>
#include <plugins/navigator/navigator_net_thread.h>
#include <plugins/navigator/motor_thread.h>

using namespace fawkes;

/** @class NavigatorPlugin plugins/navigator/navigator_plugin.h
 * The Navigator Plugin of Fawkes.
 * 
 * @author Martin Liebenberg
 */

/** Constructor. */
NavigatorPlugin::NavigatorPlugin()
  : Plugin("navigator_plugin")
{
  NavigatorThread *navigator_thread = new NavigatorThread();

  // Motor MUST be first! Navigator sends AquireControlMessage
  thread_list.push_back(new MotorThread());
  thread_list.push_back(navigator_thread);
  thread_list.push_back(new NavigatorNetworkThread(navigator_thread));
}

EXPORT_PLUGIN(NavigatorPlugin)

  /** Destructor. */
  NavigatorPlugin::~NavigatorPlugin()
{
}
