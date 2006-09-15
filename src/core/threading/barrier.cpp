
/***************************************************************************
 *  barrier.cpp - Barrier
 *
 *  Generated: Thu Sep 15 00:33:13 2006
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

#include <core/threading/barrier.h>

#include <pthread.h>

/// @cond INTERNALS
class BarrierData
{
 public:
  pthread_barrier_t barrier;
};
/// @endcond


Barrier::Barrier(unsigned int count)
{
  barrier_data = new BarrierData();
  pthread_barrier_init( &(barrier_data->barrier), NULL, count );
}


Barrier::~Barrier()
{
  pthread_barrier_destroy( &(barrier_data->barrier) );
  delete barrier_data;
}


void
Barrier::wait()
{
  pthread_barrier_wait( &(barrier_data->barrier) );
}

