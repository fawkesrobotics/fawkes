
/***************************************************************************
 *  qa_shmem.h - QA for IPC shared memory
 *
 *  Generated: Sun Sep 17 16:32:25 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

// Do not include in api reference
///@cond QA

#include <utils/ipc/shm.h>
#include <utils/ipc/shm_exceptions.h>

#include <cstring>
#include <cstdlib>
#include <signal.h>
#include <iostream>

using namespace fawkes;

#define MAGIC_TOKEN "FawkesShmemQAApp"

class QASharedMemoryHeader : public SharedMemoryHeader
{
 private:
  typedef struct {
    unsigned int type;
  } qashmem_header_t;

 public:
  QASharedMemoryHeader(unsigned int type)
  {
    header.type = type;
  }

  virtual SharedMemoryHeader *
  clone() const
  {
    QASharedMemoryHeader *qs = new QASharedMemoryHeader(header.type);
    return qs;
  }

  virtual bool operator==(const SharedMemoryHeader &s) const
  {
    const QASharedMemoryHeader *qs = dynamic_cast<const QASharedMemoryHeader *>(&s);
    return (qs && (header.type == qs->header.type));
  }

  virtual bool matches(void *memptr)
  {
    return (memcmp(memptr, &header, sizeof(qashmem_header_t)) == 0);
  }

  virtual size_t size()
  {
    return sizeof(qashmem_header_t);
  }

  virtual bool create()
  {
    return true;
  }

  virtual void initialize(void *memptr)
  {
    memcpy(memptr, (char *)&header, sizeof(qashmem_header_t));
  }

  virtual void set(void *memptr)
  {
    memcpy((char *)&header, memptr, sizeof(qashmem_header_t));
  }


  virtual void reset()
  {
  }

  virtual size_t data_size()
  {
    return 1024;
  }

 private:
  qashmem_header_t header;
};


bool quit;

void
signal_handler(int signum)
{
  quit = true;
}


int
main(int argc, char **argv)
{
  quit = false;
  signal(SIGINT, signal_handler);

  QASharedMemoryHeader *h1 = new QASharedMemoryHeader(1);

  SharedMemory *s1, *s2;

  try {
    // This will create the shared memory segment
    s1 = new SharedMemory(MAGIC_TOKEN, h1,
			  /* read only */ false,
			  /* create    */ true,
			  /* destroy   */ true);

    // This will attach to the existing shmem segment,
    // use ipcs to check
    s2 = new SharedMemory(MAGIC_TOKEN, h1,
			  /* read only */ true,
			  /* create    */ false,
			  /* destroy   */ false);
  } catch ( ShmCouldNotAttachException &e ) {
    e.print_trace();
    exit(1);
  }

  int *m1 = (int *)s1->memptr();
  int *m2 = (int *)s2->memptr();

  int i = 0;

  while ( ! quit ) {
    *m1 = i;
    std::cout << "Wrote " << *m1 << " (should be " << i
	      << ") to b1, afterwards b2 reads: " << *m2
	      << std::endl;
    usleep(500000);
    ++i;
  };

  delete s2;
  delete s1;
  delete h1;
}

/// @endcond
