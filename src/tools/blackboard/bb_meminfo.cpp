
/***************************************************************************
 *  bb_meminfo.cpp - Fawkes BlackBoard memory info
 *
 *  Generated: Fri Oct 20 13:32:38 2006
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

#include <blackboard/bbconfig.h>
#include <blackboard/internal/memory_manager.h>
#include <blackboard/internal/interface_mem_header.h>
#include <blackboard/exceptions.h>
#include <utils/system/console_colors.h>
#include <utils/time/time.h>
#include <config/sqlite.h>

#include <iostream>
#include <cstdio>

using namespace std;
using namespace fawkes;

int
main(int argc, char **argv)
{
  SQLiteConfiguration config(CONFDIR);
  config.load();

  std::string token = "";
  try {
    token = config.get_string("/fawkes/mainapp/blackboard_magic_token");
  } catch (Exception &e) {
    cout << "Could not read shared memory token for blackboard." << endl;
    cout << "BlackBoard is probably running without shared memory." << endl;
    return -1;
  }

  BlackBoardMemoryManager *memmgr;
  try {
    memmgr = new BlackBoardMemoryManager( config.get_uint("/fawkes/mainapp/blackboard_size"),
					  BLACKBOARD_VERSION,
					  /* master? */ false,
					  token.c_str());
  } catch (BBMemMgrCannotOpenException &e) {
    cout << "No BlackBoard shared memory segment found!" << endl;
    return 1;
  }

  cout << endl << cblue << "Fawkes BlackBoard Memory Info" << cnormal << endl
       << "========================================================================" << endl;

  printf("Memory Size: %s%8u%s %sB%s  BlackBoard version: %s%u%s\n"
	 "Free Memory: %s%8u%s %sB%s  Alloc. memory: %s%8u%s %sB%s  Overhang: %s%8u%s %sB%s\n"
	 "Free Chunks: %s%8u%s    Alloc. chunks: %s%8u%s\n",
 	 cdarkgray.c_str(), memmgr->memory_size(), cnormal.c_str(),
	 clightgray.c_str(), cnormal.c_str(),
	 cdarkgray.c_str(), memmgr->version(), cnormal.c_str(),
	 cdarkgray.c_str(), memmgr->free_size(), cnormal.c_str(),
	 clightgray.c_str(), cnormal.c_str(),
	 cdarkgray.c_str(), memmgr->allocated_size(), cnormal.c_str(),
	 clightgray.c_str(), cnormal.c_str(),
	 cdarkgray.c_str(), memmgr->overhang_size(), cnormal.c_str(),
	 clightgray.c_str(), cnormal.c_str(),
	 cdarkgray.c_str(), memmgr->num_free_chunks(), cnormal.c_str(),
	 cdarkgray.c_str(), memmgr->num_allocated_chunks(), cnormal.c_str());

  if ( ! memmgr->try_lock() ) {
    timeval a, b;
    gettimeofday(&a, NULL);
    cout << "Waiting for lock on shared memory.. " << flush;
    memmgr->lock();
    gettimeofday(&b, NULL);
    cout << "lock aquired. Waited " << time_diff_sec(b, a) << " seconds" << endl;
  }

  if ( memmgr->begin() == memmgr->end() ) {
    cout << "No interfaces allocated." << endl;
  } else {
    cout << endl << "Interfaces:" << endl;

    printf("%sMemSize  Overhang  Type/ID/Hash                       Serial  Ref  W/R%s\n"
	   "------------------------------------------------------------------------\n",
	   cdarkgray.c_str(), cnormal.c_str());

    interface_header_t *ih;
    BlackBoardMemoryManager::ChunkIterator cit;
    for ( cit = memmgr->begin(); cit != memmgr->end(); ++cit ) {
      if ( *cit == NULL ) {
	cout << "*cit == NULL" << endl;
	break;
      } else {
	ih = (interface_header_t *)*cit;
	char tmp_hash[__INTERFACE_HASH_SIZE * 2 + 1];
	for (size_t s = 0; s < __INTERFACE_HASH_SIZE; ++s) {
	  snprintf(&tmp_hash[s*2], 3, "%02X", ih->hash[s]);
	}
	printf("%7u  %8u  %sT%s %-32s %6u  %3u  %1d/%-3d\n%18s %sI%s %-32s\n%18s %sH%s %-32s\n",
	       cit.size(), cit.overhang(), clightgray.c_str(), cnormal.c_str(), ih->type,
	       ih->serial, ih->refcount, ih->flag_writer_active, ih->num_readers,
	       "", clightgray.c_str(), cnormal.c_str(), ih->id,
	       "", clightgray.c_str(), cnormal.c_str(), tmp_hash);
      }
    }
  }

  memmgr->unlock();

  delete memmgr;
  return 0;
}
