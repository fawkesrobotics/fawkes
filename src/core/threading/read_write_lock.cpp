
/***************************************************************************
 *  read_write_lock.cpp - Read Write Lock
 *
 *  Generated: Thu Sep 15 00:10:54 2006
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

#include <core/threading/read_write_lock.h>

#include <pthread.h>
#include <string.h>
#include <stdio.h>

/// @cond INTERNALS
class ReadWriteLockData
{
 public:
  pthread_rwlock_t rwlock;
};
/// @endcond


ReadWriteLock::ReadWriteLock(ReadWriteLockPolicy policy)
{
  rwlock_data = new ReadWriteLockData();

  pthread_rwlockattr_t attr;
  pthread_rwlockattr_init( &attr );

  switch (policy) {
  case RWLockPolicyPreferWriter:
    pthread_rwlockattr_setkind_np( &attr, PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP );
    break;
  case RWLockPolicyPreferReader:
    pthread_rwlockattr_setkind_np( &attr, PTHREAD_RWLOCK_PREFER_READER_NP );
    break;
  }

  pthread_rwlock_init( &(rwlock_data->rwlock), &attr );
}


ReadWriteLock::~ReadWriteLock()
{
  pthread_rwlock_destroy( &(rwlock_data->rwlock) );
  delete rwlock_data;
}


void
ReadWriteLock::lockForRead()
{
  pthread_rwlock_rdlock( &(rwlock_data->rwlock) );
}


void
ReadWriteLock::lockForWrite()
{
  pthread_rwlock_wrlock( &(rwlock_data->rwlock) );
}


bool
ReadWriteLock::tryLockForRead()
{
  return ( pthread_rwlock_tryrdlock( &(rwlock_data->rwlock) ) == 0 );
}


bool
ReadWriteLock::tryLockForWrite()
{
  return ( pthread_rwlock_trywrlock( &(rwlock_data->rwlock) ) == 0 );
}


void
ReadWriteLock::unlock()
{
  pthread_rwlock_unlock( &(rwlock_data->rwlock) );
}
