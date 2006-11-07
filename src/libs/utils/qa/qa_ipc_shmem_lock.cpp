
/***************************************************************************
 *  qa_shmem_lock.h - QA for protected IPC shared memory
 *
 *  Generated: Fri Oct 06 13:32:03 2006
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

// Do not include in api reference
///@cond QA

#include <utils/ipc/shm.h>
#include <utils/ipc/shm_exceptions.h>

#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <sys/types.h>
#include <sys/wait.h>

using namespace std;

#define MAGIC_TOKEN "FawkesShmemQAApp"

#define WASTETIME  \
  for ( unsigned int i = 0; i < 1000000; i++) { \
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

  virtual bool matches(void *memptr)
  {
    return (memcmp(memptr, &header, sizeof(qashmem_header_t)) == 0);
  }

  virtual unsigned int size()
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

  virtual unsigned int dataSize()
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

  SharedMemory *s1, *s2, *s3;

  try {
    // This will create the shared memory segment
    s1 = new SharedMemory(MAGIC_TOKEN, h1,
			  /* read only */ false,
			  /* create    */ true,
			  /* destroy   */ true);

    // Add protection via semaphore
    s1->addSemaphore();

    // This will attach to the existing shmem segment,
    // use ipcs to check
    s2 = new SharedMemory(MAGIC_TOKEN, h1,
    			  /* read only */ false,
    			  /* create    */ false,
    			  /* destroy   */ false);

    s3 = new SharedMemory(MAGIC_TOKEN, h1,
    			  /* read only */ false,
    			  /* create    */ false,
    			  /* destroy   */ false);

  } catch ( ShmCouldNotAttachException &e ) {
    e.printTrace();
    exit(1);
  }

  pid_t child_pid;

  if ((child_pid = fork()) == 0) {
    // child
    cout << "Child is alive" << endl;

    int *mc = (int *)s2->getMemPtr();

    while ( ! quit ) {
      int m;
      m = mc[1]; m++;
      //cout << "Child: sleeping" << endl;
      usleep(12932);
      //cout << "Child: wasting time" << endl;
      WASTETIME;
      //cout << "Child: done wasting time, setting to " << m << endl;
      mc[1] = m;
      //cout << "Child: locking" << endl;
      s2->lock();
      //cout << "Child: locked" << endl;
      m = mc[0]; m++;
      usleep(23419);
      WASTETIME;
      mc[0] = m;
      //cout << "Child: unlocking" << endl;
      s2->unlock();
      //cout << "Child: unlocked" << endl;
      std::cout << "Child:  unprotected: " << mc[1] << " protected: " << mc[0] << endl;
      usleep(1231);
    }

    delete s2;

  } else {
    // father
    cout << "Father is alive" << endl;
    int *mf = (int *)s1->getMemPtr();

    while ( ! quit ) {
      int m;
      m = mf[1]; m++;
      usleep(34572);
      WASTETIME;
      mf[1] = m;
      //cout << "Father: locking" << endl;
      s1->lock();
      //cout << "Father: locked" << endl;
      m = mf[0]; m++;
      usleep(12953);
      WASTETIME;
      mf[0] = m;
      s1->unlock();
      std::cout << "Father: unprotected: " << mf[1] << " protected: " << mf[0] << endl;
      usleep(3453);
    }

    cout << "Father: Waiting for child to exit" << endl;
    int status;
    waitpid(child_pid, &status, 0);

    delete s1;
    delete h1;
    }
}

/// @endcond
