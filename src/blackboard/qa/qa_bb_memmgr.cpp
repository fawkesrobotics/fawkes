
/***************************************************************************
 *  memory_manager.h - BlackBoard memory manager QA
 *
 *  Generated: Thu Oct 05 16:09:25 2006
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


/// @cond QA

#include <blackboard/memory_manager.h>
#include <blackboard/exceptions.h>

#include <core/exceptions/system.h>

#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <vector>

using namespace std;


bool quit = false;

void
signal_handler(int signum)
{
  quit = true;
}


#define NUM_CHUNKS 5
// 2 MB
#define MEMSIZE (2 * 1024 * 1024)

int
main(int argc, char **argv)
{

  signal(SIGINT, signal_handler);

  BlackBoardMemoryManager *mm = new BlackBoardMemoryManager( MEMSIZE,
							    1 /* version, don't care */,
							    "FawkesBBMemMgrQA" /* token */ );

  void *m[NUM_CHUNKS];

  cout << "Running basic tests" << endl;
  cout << "=========================================================================" << endl;

  mm->printPerformanceInfo();

  unsigned int free_before = mm->getMaxFreeSize();

  for (unsigned int i = 0; i < NUM_CHUNKS; ++i) {
    cout << "Allocating m[" << i << "] with " << (i+1) * 1000 << " bytes.." << flush;
    m[i] = mm->alloc( (i+1) * 1000 );
    cout << "done" << endl;
  }

  if ( mm->getMaxAllocatedSize() != (NUM_CHUNKS * 1000) ) {
    cout << "Largest chunk is not " << NUM_CHUNKS * 1000 << " bytes, error, aborting" << endl;
    delete mm;
    exit(1);
  }

  cout << "Free chunks:" << endl;
  mm->printFreeChunksInfo();
  cout << "Allocated chunks:" << endl;
  mm->printAllocatedChunksInfo();
  mm->printPerformanceInfo();

  for (unsigned int i = 0; i < NUM_CHUNKS; ++i) {
    cout << "Freeing m[" << i << "].." << flush;
    mm->free( m[i] );
    cout << "done" << endl;
  }

  if ( mm->getMaxAllocatedSize() != 0 ) {
    cout << "Largest chunk is not 0 bytes, error, aborting" << endl;
    delete mm;
    exit(2);
  }

  if ( mm->getMaxFreeSize() != free_before ) {
    cout << "Max free size after tests differe from before test, error, aborting" << endl;
    delete mm;
    exit(3);
  }

  cout << "Free chunks:" << endl;
  mm->printFreeChunksInfo();
  cout << "Allocated chunks:" << endl;
  mm->printAllocatedChunksInfo();
  mm->printPerformanceInfo();

  cout << "Basic tests finished" << endl;
  cout << "=========================================================================" << endl;

  cout << endl << "Running gremlin tests, press Ctrl-C to stop" << endl;
  cout << "=========================================================================" << endl;

  std::vector< void * >  ptrs;
  ptrs.clear();

  unsigned int modcount = 0;
  while ( ! quit ) {
    if (rand() < RAND_MAX / 2) {
      cout << "a" << flush;
      // alloc
      unsigned int s = (rand() % MEMSIZE) / 1000;
      if ( s < 20 ) {
	// min 20 bytes
	s = 20;
      }
      void *m;
      try {
	m = mm->alloc(s);
	ptrs.push_back( m );
      } catch ( OutOfMemoryException &e ) {
	cout << "Memory Manager ran out of memory, tried to allocate "
	     << s << " bytes, detailed info:" << endl;
	cout << "Free chunks:" << endl;
	mm->printFreeChunksInfo();
	cout << "Allocated chunks:" << endl;
	mm->printAllocatedChunksInfo();
	mm->printPerformanceInfo();
      }
    } else {
      cout << "f" << flush;
      // free
      if ( ptrs.size() > 0 ) {
	// there is something to delete
	unsigned int erase = rand() % ptrs.size();
	try {
	  mm->free( ptrs[erase] );
	  ptrs.erase( ptrs.begin() + erase );
	} catch ( BlackBoardMemMgrInvalidPointerException &e ) {
	  cout << "Ouch, tried to free invalid pointer" << endl;
	  cout << "Allocated chunks:" << endl;
	  mm->printAllocatedChunksInfo();
	  printf("Pointer tried to free: 0x%x\n", (unsigned int)ptrs[erase]);
	}
      }
    }

    try {
      mm->check();
    } catch ( BBInconsistentMemoryException &e ) {
      cout << "Inconsistent memory found, printing exception trace" << endl;
      e.printTrace();
      cout << "Free chunks:" << endl;
      mm->printFreeChunksInfo();
      cout << "Allocated chunks:" << endl;
      mm->printAllocatedChunksInfo();
      mm->printPerformanceInfo();
      quit = true;
    }

    if ( modcount % 10 == 0 ) {
      cout << endl;
      mm->printPerformanceInfo();
      if ( mm->getOverhangSize() > 0 ) {
	cout << "Overhang detected, allocated chunks:" << endl;
	mm->printAllocatedChunksInfo();
      }
    }
    ++modcount;
    usleep(0);
  }

  delete mm;
}


/// @endcond
