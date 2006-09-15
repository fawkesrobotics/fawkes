
/***************************************************************************
 *  mutex.cpp - implementation of mutex, based on pthreads
 *
 *  Generated: Thu Sep 14 17:03:57 2006
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

#include <core/threading/mutex.h>
#include <core/threading/mutex_data.h>

#include <pthread.h>

Mutex::Mutex()
{
  mutex_data = new MutexData();
  pthread_mutex_init(&(mutex_data->mutex), NULL);
}

Mutex::~Mutex()
{
  pthread_mutex_destroy(&(mutex_data->mutex));
  delete mutex_data;
  mutex_data = NULL;
}


void
Mutex::lock()
{
  pthread_mutex_lock(&(mutex_data->mutex));
}


bool
Mutex::tryLock()
{
  return (pthread_mutex_trylock(&(mutex_data->mutex)) == 0);
}


void
Mutex::unlock()
{
  pthread_mutex_unlock(&(mutex_data->mutex));
}
