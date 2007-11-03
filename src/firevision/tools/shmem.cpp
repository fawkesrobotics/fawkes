
/***************************************************************************
 *  shmem.cpp - Shared memory management tool
 *
 *  Generated: Mon Jan 16 22:51:34 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/ipc/shm_image.h>
#include <fvutils/ipc/shm_lut.h>
#include <utils/system/argparser.h>
#include <iostream>

using namespace std;

int
main(int argc, char **argv)
{

  ArgumentParser *argp = new ArgumentParser(argc, argv, "c::Hl::");
  bool action_done = false;

  if ( argp->has_arg("H") ) {
    // Show usage note
    cout << endl << "Usage: " << argv[0] << " [-h] [-c] [-c[t]] [-l] [-m]" << endl
	 << " -h     Show this help message" << endl
	 << " -c[t]  Cleanup (remove all FireVision related shmem segments of given type)"
	 << endl
	 << " -l[t]  List shared memory segments of given type" << endl
	 << endl
	 << "        [t] type is a combination of" << endl
	 << "          i  images" << endl
	 << "          l  lookup tables" << endl
	 << "        or empty in which case all known shared memory segments are mangled"
	 << endl << endl
	 << "By default all known shared memory segments are listed" << endl
	 << endl;
    action_done = true;
  } else {
    if ( argp->has_arg("c") ) {
      const char *tmp;
      if ( (tmp = argp->arg("c")) != NULL) {
	if ( strchr(tmp, 'i') != NULL) {
	  SharedMemoryImageBuffer::cleanup();
	}
	if ( strchr(tmp, 'l') != NULL) {
	  SharedMemoryLookupTable::cleanup();
	}
      } else {
	SharedMemoryImageBuffer::cleanup();
	SharedMemoryLookupTable::cleanup();
      }

      action_done = true;
    }
    if ( argp->has_arg("l") ) {
      const char *tmp;
      if ( (tmp = argp->arg("l")) != NULL) {
	if ( strchr(tmp, 'i') != NULL) {
	  SharedMemoryImageBuffer::list();
	}
	if ( strchr(tmp, 'l') != NULL) {
	  SharedMemoryLookupTable::list();
	}
      } else {
	SharedMemoryImageBuffer::list();
	SharedMemoryLookupTable::list();
      }

      action_done = true;
    }
  }

  if (! action_done) {
    SharedMemoryImageBuffer::list();
    cout << endl;
    SharedMemoryLookupTable::list();
  }

  cout << endl;
}
