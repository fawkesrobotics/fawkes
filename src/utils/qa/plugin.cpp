
/***************************************************************************
 *  plugin.cpp - QA Application for dynamic modules and plugins
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

#include <utils/system/dynamic_module/module_dl.h>
#include <utils/system/dynamic_module/module_manager_template.h>
#include <core/plugin.h>
#include <utils/plugin/plugin_loader.h>

#include <iostream>

using namespace std;


bool
test_plugin(Plugin *p)
{	
  cout << "Plugin name: " << p->getName() << endl;
  cout << "Plugin type: " << p->getType() << endl;

  return true;
}


bool
test_module(Module *m)
{
  bool success = true;
  if ( ! m->open() ) {
    cout << "Could not open module" << endl;
    success = false;
  } else {
    cout << "Successfully opened module" << endl;
  
    if ( ! m->hasSymbol("plugin_factory") ) { // "plugin_factory"
      cout << "Doh, symbol not found" << endl;
      success = false;
    } else {
      cout << "Yeah, we got the symbol" << endl;

      PluginFactoryFunc pff = (PluginFactoryFunc)m->getSymbol("plugin_factory");
      PluginDestroyFunc pdf = (PluginDestroyFunc)m->getSymbol("plugin_destroy");

      if ( (pff != NULL) && (pdf != NULL) ) {
	Plugin *p = pff();

	success = test_plugin(p);

	pdf(p);
	p = NULL;

      } else {
	success = false;
	if ( pff == NULL ) {
	  cout << "pff == NULL" << endl;
	}
	if ( pdf == NULL ) {
	  cout << "pdf == NULL" << endl;
	}
      }
    }
    m->close();
  }

  return success;
}


int
main(int argc, char **argv)
{
  // Load just the test module

  bool success = true;

  cout << "Running plain module tests" << endl;
  ModuleDL *m = new ModuleDL(PLUGINDIR"/test_plugin.so");
  success = test_module(m);
  delete m;
  if ( success ) {
    cout << "SUCCESSFULLY tested plain module" << endl;
  } else {
    cout << "FAILED plain module tests, aborting further tests" << endl;
    return -1;
  }

  success = true;
  cout << endl << endl << "Running ModuleManagerTemplate tests" << endl;
  ModuleManagerTemplate<ModuleDL> mm(PLUGINDIR);
  Module *mod = mm.openModule("test_plugin.so");
  if ( mod == NULL ) {
    cout << "Failed to retrieve module from manager" << endl;
    success = false;
  } else {
    cout << "Retrieved module from module manager" << endl;
  }

  success = test_module(mod);

  cout << "Testing ref count" << endl;
  cout << "RefCount (should be 1): " << mod->getRefCount() << endl;
  cout << "Retrieving module twice, again" << endl;
  mm.openModule("test_plugin.so");
  mm.openModule("test_plugin.so");
  cout << "RefCount (should be 3): " << mod->getRefCount() << endl;
  cout << "Closing module twice" << endl;
  mm.closeModule(mod);
  mm.closeModule(mod);
  cout << "RefCount (should be 1): " << mod->getRefCount() << endl;
  cout << "Finally closing module" << endl;
  mm.closeModule(mod);
  if ( mm.moduleOpened("test_plugin.so") ) {
    cout << "Plugin still opened, bug!" << endl;
    success = false;
  } else {
    cout << "Plugin has been unloaded from module manager" << endl;
  }


  if ( success ) {
    cout << "SUCCESSFULLY tested module manager" << endl;
  } else {
    cout << "FAILED module manager tests, aborting further tests" << endl;
    return 2;
  }


  success = true;
  cout << endl << endl << "Running PluginLoader tests" << endl;
  PluginLoader *pl = new PluginLoader(PLUGINDIR);

  Plugin *p;
  if ( ! pl->load("test_plugin", p) ) {
    cout << "Could not load plugin" << endl;
    success = false;
  } else {
    success = test_plugin(p);
  }
  pl->unload(p);

  delete pl;
  if ( success ) {
    cout << "SUCCESSFULLY tested PluginLoader" << endl;
  } else {
    cout << "FAILED module manager tests, aborting further tests" << endl;
    return 3;
  }


  return 0;
}
