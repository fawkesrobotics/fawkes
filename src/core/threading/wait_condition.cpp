
/***************************************************************************
 *  wait_condition.cpp - condition variable implementation
 *
 *  Generated: Thu Sep 14 21:43:30 2006
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

#include <core/threading/wait_condition.h>
#include <core/threading/mutex_data.h>

#include <pthread.h>

/// @cond INTERNALS
class WaitConditionData
{
 public:
  pthread_cond_t cond;
  pthread_mutex_t mutex;
};
/// @endcond


WaitCondition::WaitCondition()
{
  cond_data = new WaitConditionData();
  pthread_cond_init( &(cond_data->cond), NULL );
  pthread_mutex_init( &(cond_data->mutex), NULL );
}


WaitCondition::~WaitCondition()
{
  pthread_cond_destroy( &(cond_data->cond) );
  pthread_mutex_destroy( &(cond_data->mutex) );
  delete cond_data;
  cond_data = NULL;
}


bool
WaitCondition::wait(Mutex *mutex, unsigned int timeout_sec, unsigned int timeout_nanosec)
{
  if ( (timeout_sec > 0) || (timeout_nanosec > 0) ) {
    struct timespec ts = { timeout_sec, timeout_nanosec };
    int err;
    err = pthread_cond_timedwait( &(cond_data->cond), &(mutex->mutex_data->mutex), &ts );
    if ( err == 0 ) {
      return true;
    } else {
      return false;
    }
  } else {
    pthread_cond_wait( &(cond_data->cond), &(mutex->mutex_data->mutex) );
    return true;
  }
}


void
WaitCondition::wakeOne()
{
  pthread_mutex_lock( &(cond_data->mutex) );
  pthread_cond_signal( &(cond_data->cond) );
  pthread_mutex_unlock( &(cond_data->mutex) );
}


void
WaitCondition::wakeAll()
{
  pthread_mutex_lock( &(cond_data->mutex) );
  pthread_cond_broadcast( &(cond_data->cond) );
  pthread_mutex_unlock( &(cond_data->mutex) );
}

