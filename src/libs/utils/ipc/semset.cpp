
/***************************************************************************
 *  semset.cpp - ICP semaphore sets
 *
 *  Generated: Tue Sep 19 15:02:32 2006
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

#include <utils/ipc/semset.h>
#include <utils/ipc/sem_exceptions.h>
#include <core/exceptions/system.h>

#include <errno.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <limits.h>

namespace fawkes {


/// @cond INTERNALS
class SemaphoreSetData
{
 public:
  key_t   key;
  int     semid;
  int     semflg;
  int     num_sems;
};

#ifdef _SEM_SEMUN_UNDEFINED
union semun
{
  int val;                   /* value for SETVAL */
  struct semid_ds *buf;      /* buffer for IPC_STAT & IPC_SET */
  unsigned short int *array; /* array for GETALL & SETALL */
  struct seminfo *__buf;     /* buffer for IPC_INFO */
};
#endif

/// @endcond


/** @class SemaphoreSet utils/ipc/semset.h
 * IPC semaphore set.
 * This class handles semaphore sets. A semaphore is a tool to control access
 * to so-called critical sections. It is used to ensure that only a single
 * process at a time is in the critical section or modifying shared data
 * to avoid corruption.
 *
 * Semaphores use a single integer as the semaphore value. It denotes the
 * number of resources that are available for the given semaphore. For
 * example if you have two cameras on a robot you may have a value of two
 * for the semaphore value. If the value reaches zero no more resources are
 * available. You will have to wait until more resources are freed again.
 *
 * Now these individual semaphores are bundled to sets of semaphores. This is
 * useful since there are situations where you want different semaphores
 * for different operations on the shared resource. In the case of a shared
 * memory segment for instance you could have one semaphore for reading
 * and one for writing.
 *
 * @ingroup IPC
 * @see qa_ipc_semset.cpp
 * @author Tim Niemueller
 *
 *
 * @var SemaphoreSet::destroy_on_delete
 * Destroy this semaphore on delete?
 */


/** Constructor.
 * Creates a new semaphore set. Will try to open the semaphore if it does
 * exist. Tries to create if create is assured.
 * @param path Path to generate the id from
 * @param id Additional info for id.
 * @param num_sems Number of semaphores to generate in this set. Only used
 *                 if semaphore set did not already exist and create is
 *                 assured.
 * @param destroy_on_delete If true semaphore set is destroyed if instance
 *                          is deleted.
 * @param create If true semaphore set is created if it does not exist.
 */
SemaphoreSet::SemaphoreSet(const char *path, char id,
			   int num_sems,
			   bool create, bool destroy_on_delete)
{
  data = new SemaphoreSetData();

  if ( num_sems < 0 ) {
    num_sems = - num_sems;
  }

  this->destroy_on_delete = destroy_on_delete;
  data->num_sems = num_sems;

  data->semflg = 0666;
  if (create) {
    data->semflg |= IPC_CREAT;
  }

  data->key   = ftok(path, id);
  data->semid = semget(data->key, num_sems, data->semflg);

}


/** Constructor.
 * Creates a new semaphore set. Will try to open the semaphore if it does
 * exist. Tries to create if create is assured.
 * @param key Key of semaphore set as printed by ipcs.
 * @param num_sems Number of semaphores to generate in this set. Only used
 *                 if semaphore set did not already exist and create is
 *                 assured.
 * @param destroy_on_delete If true semaphore set is destroyed if instance
 *                          is deleted.
 * @param create If true semaphore set is created if it does not exist.
 */
SemaphoreSet::SemaphoreSet(int key,
			   int num_sems,
			   bool create, bool destroy_on_delete)
{
  data = new SemaphoreSetData();

  if ( num_sems < 0 ) {
    num_sems = - num_sems;
  }

  this->destroy_on_delete = destroy_on_delete;
  data->num_sems = num_sems;

  data->semflg = 0666;
  if (create) {
    data->semflg |= IPC_CREAT;
  }

  data->key   = key;
  data->semid = semget(data->key, num_sems, data->semflg);

  if ( data->semid == -1 ) {
    throw Exception(errno, "Creating the semaphore set failed, maybe key does not exist");
  }
}


/** Constructor.
 * Creates a new semaphore set with a new ID supplied by the system. The
 * id can be queried with getID.
 * @param num_sems Number of semaphores to generate in this set. Only used
 *                 if semaphore set did not already exist and create is
 *                 assured.
 * @param destroy_on_delete If true semaphore set is destroyed if instance
 *                          is deleted.
 */
SemaphoreSet::SemaphoreSet(int num_sems,
			   bool destroy_on_delete)
{
  data = new SemaphoreSetData();

  if ( num_sems < 0 ) {
    num_sems = - num_sems;
  }

  this->destroy_on_delete = destroy_on_delete;
  data->num_sems = num_sems;

  data->semflg = 0666;
  data->semflg |= IPC_CREAT;
  data->semflg |= IPC_EXCL;  

  for (data->key = 1; data->key < INT_MAX; data->key++) {
    data->semid = semget(data->key, num_sems, data->semflg);
    if ( data->semid != -1 ) {
      // valid semaphore found
      break;
    }
  }
}


/** Destructor */
SemaphoreSet::~SemaphoreSet()
{
  if ((data->semid != -1) && destroy_on_delete) {
    semctl(data->semid, 0, IPC_RMID, 0);
  }
  delete data;
}


/** Check if the semaphore set is valid.
 * If the queue could not be opened yet (for example if you gave create=false to the
 * constructor) isValid() will try to open the queue.
 * @return This method returns false if the message queue could not be opened
 *         or if it has been closed, it returns true if messages can be sent or received.
 */
bool
SemaphoreSet::valid()
{
  if (data->semid == -1) {
    data->semid = semget(data->key, data->num_sems, data->semflg);
    if (data->semid == -1) {
      return false;
    } else {
      struct semid_ds semds;
      union semun s;
      s.buf = &semds;
      if (semctl(data->semid, 0, IPC_STAT, s) != -1) {
	return true;
      } else {
	data->semid = -1;
	return false;
      }
    }
  } else {
    struct semid_ds semds;
    union semun s;
    s.buf = &semds;
    if (semctl(data->semid, 0, IPC_STAT, s) != -1) {
      return true;
    } else {
      data->semid = -1;
      return false;
    }
  }
}

/** Lock resources on the semaphore set.
 * Locks num resources on semaphore sem_num.
 * @param sem_num The semaphore number in the set
 * @param num How many resources to lock? Positive number.
 * @exception InterruptedException Operation was interrupted (for instance by a signal)
 * @exception SemCannotLockException Semaphore cannot be locked
 * @exception SemInvalidException Semaphore set is invalid
 */
void
SemaphoreSet::lock(unsigned short sem_num, short num)
{
  if ( data->semid == -1 )  throw SemInvalidException();

  struct sembuf sop;
  sop.sem_num = sem_num;
  sop.sem_op  = (short)((num <= 0) ? num : -num);
  sop.sem_flg = 0;
  if ( semop(data->semid, &sop, 1) != 0 ) {
    if ( errno == EINTR ) throw InterruptedException();
    else                  throw SemCannotLockException();
  }
}


/** Try to lock resources on the semaphore set.
 * @param sem_num The semaphore number in the set
 * @param num How many resources to lock? Positive number.
 * @return true, if the semaphore could be locked, false otherwise
 * @exception InterruptedException Operation was interrupted (for instance by a signal)
 * @exception SemCannotLockException Semaphore cannot be locked
 * @exception SemInvalidException Semaphore set is invalid
 */
bool
SemaphoreSet::try_lock(unsigned short sem_num, short num)
{
  if ( data->semid == -1 )  throw SemInvalidException();

  struct sembuf sop;
  sop.sem_num = sem_num;
  sop.sem_op  = (short)((num <= 0) ? num : -num);
  sop.sem_flg = IPC_NOWAIT;
  if ( semop(data->semid, &sop, 1) != 0 ) {
    if (errno == EAGAIN) {
      return false;
    } else if ( errno == EINTR ) {
      throw InterruptedException();
    } else {
      throw SemCannotLockException();
    }
  }
  return true;
}


/** Unlock resources on the semaphore set.
 * @param sem_num The semaphore number in the set
 * @param num How many resources to unlock? Negative number.
 * @exception InterruptedException Operation was interrupted (for instance by a signal)
 * @exception SemCannotUnlockException Semaphore cannot be unlocked
 * @exception SemInvalidException Semaphore set is invalid
 */
void
SemaphoreSet::unlock(unsigned short sem_num, short num)
{
  if ( data->semid == -1 )  throw SemInvalidException();

  struct sembuf sop;
  sop.sem_num = sem_num;
  sop.sem_op  = (short)((num >= 0) ? num : -num);
  sop.sem_flg = 0;
  if ( semop(data->semid, &sop, 1) != 0 ) {
    if ( errno == EINTR ) throw InterruptedException();
    else                  throw SemCannotUnlockException();
  }
}


/** Set the semaphore value.
 * @param sem_num The semaphore number in the set
 * @param val The value to set
 * @exception SemCannotSetValException Cannot set value
 */
void
SemaphoreSet::set_value(int sem_num, int val)
{
  if ( data->semid == -1 )  throw SemInvalidException();

  union semun s;
  s.val = val;

  if ( semctl(data->semid, sem_num, SETVAL, s) == -1 ) {
    throw SemCannotSetValException();
  }
}


/** Get the semaphore value.
 * @param sem_num The semaphore number in the set
 * @return value of the semaphore
 * @exception SemInvalidException Semaphore set is invalid
 */
int
SemaphoreSet::get_value(int sem_num)
{
  if ( data->semid == -1 )  throw SemInvalidException();

  return ( semctl(data->semid, sem_num, GETVAL, 0) != 0 );
}


/** Get key of semaphore.
 * @return Key of semaphore as listed by ipcs.
 */
int
SemaphoreSet::key()
{
  return data->key;
}


/** Set if semaphore set should be destroyed on delete.
 * If this is set to true the semaphore set is destroyed from the system if this
 * instance is deleted.
 * @param destroy set to true, if semaphore set should be destroyed on delete,
 * false otherwise
 */
void
SemaphoreSet::set_destroy_on_delete(bool destroy)
{
  destroy_on_delete = destroy;
}


/* ==================================================================
 * STATICs
 */

/** Get a non-zero free key
 * Scans the key space sequentially until a non-zero unused key is found. Not
 * that using this can cause a race-condition. You are in most cases better off
 * using the appropriate constructor that automatically finds a free key.
 * @return 0, if no free key could be found, otherwise the non-zero unused key
 */
int
SemaphoreSet::free_key()
{
  bool found = false;
  int key;
  int semid;
  for (key = 1; key < INT_MAX; ++key) {
    semid = semget(key, 1, IPC_CREAT | IPC_EXCL);
    if ( semid != -1 ) {
      // valid semaphore found
      semctl(semid, 0, IPC_RMID, 0);
      found = true;
      break;
    }
  }
  return (found ? key : 0);
}


/** Destroy a semaphore set.
 * Destroy the semaphore denoted by key. No tests are done if some other
 * process is using this semaphore. Use with care!
 * @param key key of the semaphore set
 */
void
SemaphoreSet::destroy(int key)
{
  int semid = semget(key, 0, 0);
  if ( semid == -1 ) return;
  semctl(semid, 0, IPC_RMID, 0);
}


} // end namespace fawkes
