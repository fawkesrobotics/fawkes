 
/***************************************************************************
 *  battery_plugin.cpp - Fawkes Battery Plugin
 *
 *  Generated: Tue Jan 29 11:56:28 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <plugins/battery/battery_plugin.h>
#include <plugins/battery/battery_thread.h>

/** @class BatteryPlugin plugins/battery/battery_plugin.h
 * Fawkes plugin that reads out the battery status.
 *
 * @author Daniel Beck
 */

/** Constructor. */
BatteryPlugin::BatteryPlugin()
  : Plugin("battery_plugin")
{
  thread_list.push_back(new BatteryThread());
}


EXPORT_PLUGIN(BatteryPlugin)
