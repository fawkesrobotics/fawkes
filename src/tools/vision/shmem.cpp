
/***************************************************************************
 *  shmem.cpp - Shared memory management tool
 *
 *  Generated: Mon Jan 16 22:51:34 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/ipc/shm_image.h>
#include <fvutils/ipc/shm_lut.h>
#include <utils/system/argparser.h>
#include <fvutils/writers/fvraw.h>

#include <iostream>
#include <cstring>
#include <cstdio>

using namespace std;
using namespace fawkes;
using namespace firevision;

int
main(int argc, char **argv)
{

  ArgumentParser *argp = new ArgumentParser(argc, argv, "c::hl::i:");
  bool action_done = false;

  if ( argp->has_arg("h") ) {
    // Show usage note
    cout << endl << "Usage: " << argv[0] << " [-h] [-c] [-c[t]] [-l] [-i image_id] [file]" << endl
	 << " -h     Show this help message" << endl
	 << " -i id  Save image ID to file" << endl
	 << " -c[t]  Cleanup (remove all FireVision related shmem segments of given type)"
	 << endl
	 << " -l[t]  List shared memory segments of given type" << endl
	 << endl
	 << "        [t] type is a combination of" << endl
	 << "          i  images" << endl
	 << "          l  lookup tables" << endl
	 << "        or empty in which case all known shared memory segments are mangled" << endl
	 << endl
	 << "        [file] is a file name. Content depends on the action. The possibilities are: " << endl
	 << "        for  -i   File where the saved image is stored" << endl
	 << endl
	 << "By default all known shared memory segments are listed" << endl
	 << endl;
    action_done = true;
  } else {
    if ( argp->has_arg("i") ) {
      if ( argp->num_items() == 0 ) {
	printf("You have to give a file name where to store the image\n");
      } else {
	const char *image_id = argp->arg("i");

	try {
	  SharedMemoryImageBuffer *b = new SharedMemoryImageBuffer(image_id);
	  
	  FvRawWriter *w = new FvRawWriter(argp->items()[0], b->width(), b->height(),
					 b->colorspace(), b->buffer());
	  w->write();
	  delete w;
	  delete b;
	  printf("Image '%s' saved to %s\n", image_id, argp->items()[0]);
	} catch (Exception &e) {
	  printf("Failed top save image\n");
	  e.print_trace();
	}
      }
    }
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
