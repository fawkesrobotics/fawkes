
/***************************************************************************
 *  loader.cpp - interface loader qa
 *
 *  Created: Tue Oct 17 12:15:28 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

/// @cond QA

#include <interface/interface.h>

#include <utils/system/dynamic_module/module_dl.h>
#include <utils/system/argparser.h>

#include <iostream>

#include <cstdlib>
#include <cstring>

using namespace std;


int
main(int argc, char **argv)
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "");

  ModuleDL *mod = new ModuleDL(LIBDIR"/libinterfaces.so");
  mod->open();

  const vector<const char *> items = argp->items();

  if ( items.size() == 0 ) {
    cout << "Usage: " << argp->program_name() << " interface_type [interface_type...]"
	 << endl;
  }

  for (vector<const char *>::const_iterator i = items.begin(); i != items.end(); ++i) {
    // try to load mod
    const char *type = (*i);
    char *generator_name = (char *)malloc(strlen("new") + strlen(type) + 1);
    sprintf(generator_name, "new%s", type);
    if ( ! mod->hasSymbol(generator_name) ) {
      free(generator_name);
      cout << "Could not find factory for interface of type " << *i << endl;
      continue;
    }

    InterfaceFactoryFunc iff = (InterfaceFactoryFunc)mod->getSymbol(generator_name);
    Interface *iface = iff();

    free(generator_name);

    cout << "Interface of type " << *i << " loaded successfully. Unloading." << endl;

    char *destroyer_name = (char *)malloc(strlen("delete") + strlen(type) + 1);
    sprintf(destroyer_name, "delete%s", type);
    if ( ! mod->hasSymbol(destroyer_name) ) {
      free(destroyer_name);
      cout << "Could not find destroyer for interface of type " << *i << endl;
      continue;
    }

    InterfaceDestroyFunc idf = (InterfaceDestroyFunc)mod->getSymbol(destroyer_name);
    idf(iface);
    free(destroyer_name);
  }

  mod->close();
  delete mod;
}


/// @endcond
