
/***************************************************************************
 *  kicker_plugin.cpp - Fawkes Kicker Plugin
 *
 *  Generated: Fri May 11 14:17:56 2007
 *  Copyright  2007  Daniel Beck
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/kicker/kicker_plugin.h>
#include <plugins/kicker/kicker_thread.h>

/** @class KickerPlugin plugins/kicker/kicker_plugin.h
 * Fawkes plugin that allows to control the kicker.
 * The communication with the IOWarrior is encapsulated in the KickerControl class.
 *
 * @author Daniel Beck
 */

/** Constructor. */
KickerPlugin::KickerPlugin()
  : Plugin(Plugin::MOTION, "kicker_plugin")
{
  thread_list.push_back(new KickerThread(BlockedTimingAspect::WAKEUP_HOOK_ACT, "KickerThread"));
}


/** Destructor. */
KickerPlugin::~KickerPlugin()
{
  for (ThreadList::iterator i = thread_list.begin(); i != thread_list.end(); ++i) {
    delete *i;
  }
}


/** Plugin factory function for this plugin.                                    
 * @return an instance of ExamplePlugin                                         
 */
extern "C"
Plugin *
plugin_factory()
{
  return new KickerPlugin();
}


/** Plugin destruction function for this plugin.                                
 * @param plugin The plugin that is to be destroyed. Do not use this plugin     
 *        afterwards                                                            
 */
extern "C"
void
plugin_destroy(Plugin *plugin)
{
  delete plugin;
}
