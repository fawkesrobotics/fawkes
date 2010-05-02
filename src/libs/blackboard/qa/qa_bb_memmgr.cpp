
/***************************************************************************
 *  memory_manager.h - BlackBoard memory manager QA
 *
 *  Generated: Thu Oct 05 16:09:25 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */


/// @cond QA

#include <blackboard/internal/memory_manager.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>

#include <core/exceptions/system.h>

#include <signal.h>
#include <cstdlib>
#include <cstdio>

#include <iostream>
#include <vector>

using namespace std;
using namespace fawkes;


bool quit = false;

void
signal_handler(int signum)
{
  quit = true;
}


#define NUM_CHUNKS 5
#define BLACKBOARD_MEMORY_SIZE 2 * 1024 * 1024

int
main(int argc, char **argv)
{

  signal(SIGINT, signal_handler);

  //  BlackBoardMemoryManager *mm = new BlackBoardMemoryManager( BLACKBOARD_MEMORY_SIZE,
  //							    BLACKBOARD_VERSION,
  //							    "FawkesBBMemMgrQA" /* token */ );
  BlackBoardMemoryManager *mm = new BlackBoardMemoryManager(BLACKBOARD_MEMORY_SIZE);

  void *m[NUM_CHUNKS];

  cout << "Running basic tests" << endl;
  cout << "=========================================================================" << endl;

  mm->print_performance_info();

  unsigned int free_before = mm->max_free_size();

  for (unsigned int i = 0; i < NUM_CHUNKS; ++i) {
    cout << "Allocating m[" << i << "] with " << (i+1) * 1000 << " bytes.." << flush;
    m[i] = mm->alloc( (i+1) * 1000 );
    cout << "done" << endl;
  }

  if ( mm->max_allocated_size() != (NUM_CHUNKS * 1000) ) {
    cout << "Largest chunk is not " << NUM_CHUNKS * 1000 << " bytes, error, aborting" << endl;
    delete mm;
    exit(1);
  }

  cout << "Free chunks:" << endl;
  mm->print_free_chunks_info();
  cout << "Allocated chunks:" << endl;
  mm->print_allocated_chunks_info();
  mm->print_performance_info();

  for (unsigned int i = 0; i < NUM_CHUNKS; ++i) {
    cout << "Freeing m[" << i << "].." << flush;
    mm->free( m[i] );
    cout << "done" << endl;
  }

  if ( mm->max_allocated_size() != 0 ) {
    cout << "Largest chunk is not 0 bytes, error, aborting" << endl;
    delete mm;
    exit(2);
  }

  if ( mm->max_free_size() != free_before ) {
    cout << "Max free size after tests differe from before test, error, aborting" << endl;
    delete mm;
    exit(3);
  }

  cout << "Free chunks:" << endl;
  mm->print_free_chunks_info();
  cout << "Allocated chunks:" << endl;
  mm->print_allocated_chunks_info();
  mm->print_performance_info();

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
      unsigned int s = (rand() % BLACKBOARD_MEMORY_SIZE) / 1000;
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
	mm->print_free_chunks_info();
	cout << "Allocated chunks:" << endl;
	mm->print_allocated_chunks_info();
	mm->print_performance_info();
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
	  mm->print_allocated_chunks_info();
	  printf("Pointer tried to free: 0x%lx\n", (long unsigned int)ptrs[erase]);
	}
      }
    }

    try {
      mm->check();
    } catch ( BBInconsistentMemoryException &e ) {
      cout << "Inconsistent memory found, printing exception trace" << endl;
      e.print_trace();
      cout << "Free chunks:" << endl;
      mm->print_free_chunks_info();
      cout << "Allocated chunks:" << endl;
      mm->print_allocated_chunks_info();
      mm->print_performance_info();
      quit = true;
    }

    if ( modcount % 10 == 0 ) {
      cout << endl;
      mm->print_performance_info();
      if ( mm->overhang_size() > 0 ) {
	cout << "Overhang detected, allocated chunks:" << endl;
	mm->print_allocated_chunks_info();
      }
      // sleep(10);
    }
    ++modcount;
    usleep(0);
  }

  delete mm;
}


/// @endcond
