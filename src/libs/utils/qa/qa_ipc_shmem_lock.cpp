
/***************************************************************************
 *  qa_shmem_lock.h - QA for protected IPC shared memory
 *
 *  Generated: Fri Oct 06 13:32:03 2006
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
#include <sys/types.h>
#include <sys/wait.h>

using namespace std;
using namespace fawkes;

#define MAGIC_TOKEN "FawkesShmemQAApp"

#define WASTETIME  \
  for ( unsigned int i = 0; i < 50000000; i++) { \
    unsigned int j;				\
    j = i + i;					\
  }


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


void
do_child(unsigned int child_id, QASharedMemoryHeader *header)
{
  cout << "Child " << child_id << " is alive" << endl;

  // This will attach to the existing shmem segment,
  // use ipcs to check
  SharedMemory *sr = new SharedMemory(MAGIC_TOKEN, header,
				      /* read only */ false,
				      /* create    */ false,
				      /* destroy   */ false);


  int *mc = (int *)sr->memptr();
    
  cout << "Child " << child_id << " entering loop" << endl;
  while ( ! quit ) {
    int m;
    m = mc[1]; m++;
    //cout << "Child: sleeping" << endl;
    usleep(12932);
    //cout << "Child: wasting time" << endl;
    WASTETIME;
    //cout << "Child: done wasting time, setting to " << m << endl;
    // mc[1] = m;
    cout << "Child " << child_id << ": locking (read)" << endl;
    sr->lock_for_read();
    cout << "Child " << child_id << ": locked (read)" << endl;
    m = mc[0]; m++;
    usleep(23419);
    WASTETIME;
    cout << "Child " << child_id << ": unlocking (read)" << endl;
    sr->unlock();

    cout << "Child " << child_id << ": locking (write)" << endl;
    sr->lock_for_write();
    cout << "Child " << child_id << ": locked (write)" << endl;
    mc[0] = m;
    cout << "Child " << child_id << ": unlocking (write)" << endl;
    sr->unlock();

    //cout << "Child: unlocked" << endl;
    // std::cout << "Child " << child_id << ":  unprotected: " << mc[1] << " protected: " << mc[0] << endl;
    usleep(1231);
  }

  cout << "Child " << child_id << " exiting" << endl;

  delete sr;
}


int
main(int argc, char **argv)
{
  quit = false;
  signal(SIGINT, signal_handler);

  QASharedMemoryHeader *h1 = new QASharedMemoryHeader(1);

  SharedMemory *sw;

  cout << "Use the locking/locked comments to verify!" << endl;

  try {
    cout << "Creating shared memory segment" << endl;
    // This will create the shared memory segment
    sw = new SharedMemory(MAGIC_TOKEN, h1,
			  /* read only */ false,
			  /* create    */ true,
			  /* destroy   */ true);

    // Add protection via semaphore
    cout << "Adding semaphore set for protection" << endl;
    sw->add_semaphore();

  } catch ( ShmCouldNotAttachException &e ) {
    e.print_trace();
    exit(1);
  }

  pid_t child_pid;


  if ((child_pid = fork()) == 0) {
    // child == reader
    do_child(1, h1);
  } else {
    if ((child_pid = fork()) == 0) {
      // child == reader
      do_child(2, h1);
    } else {
      // father
      cout << "Father (Writer) is alive" << endl;
      int *mf = (int *)sw->memptr();

      while ( ! quit ) {
	int m;
	m = mf[1]; m++;
	usleep(34572);
	WASTETIME;
	mf[1] = m;
	cout << "Father: locking" << endl;
	sw->lock_for_write();
	cout << "Father: locked" << endl;
	m = mf[0]; m++;
	usleep(12953);
	WASTETIME;
	mf[0] = m;
	sw->unlock();
	std::cout << "Father: unprotected: " << mf[1] << " protected: " << mf[0] << endl;
	usleep(3453);
      }

      cout << "Father: Waiting for child to exit" << endl;
      int status;
      waitpid(child_pid, &status, 0);

      delete sw;
      delete h1;
    }
  }
}

/// @endcond
