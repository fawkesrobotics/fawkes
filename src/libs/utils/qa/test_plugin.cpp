
/***************************************************************************
 *  test_plugin.cpp - QA test plugin
 *
 *  Generated: Wed Aug 23 17:00:00 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <core/plugin.h>

#include <stdio.h>

/** Simple test plugin for QA application.
 */
class TestPlugin : public Plugin {

 public:
  /** Constructor, prints out info message
   */
  TestPlugin() : Plugin("TestPlugin")
  {
    printf("TestPlugin constructor called\n");
  }

  /** Destrcutor, prints out info message
   */
  ~TestPlugin()
  {
    printf("TestPlugin destructor called\n");
  }
};


/** Plugin factory function for this plugin.
 * @return an instance of TestPlugin
 */
extern "C"
Plugin *
plugin_factory()
{
  return new TestPlugin();
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

