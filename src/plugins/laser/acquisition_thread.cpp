
/***************************************************************************
 *  acqusition_thread.cpp - Thread that retrieves the laser data
 *
 *  Created: Wed Oct 08 13:42:32 2008
 *  Copyright  2008-2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "acquisition_thread.h"

#include <core/threading/mutex.h>

#include <limits>
#include <cstring>
#include <cstdlib>

using namespace fawkes;

/** @class LaserAcquisitionThread "acquisition_thread.h"
 * Laser acqusition thread.
 * Interface for different laser types.
 * @author Tim Niemueller
 *
 * @fn void LaserAcquisitionThread::pre_init(fawkes::Configuration *config, fawkes::Logger *logger) = 0;
 * Pre initialization.
 * This method is called by the sensor thread for pre-initialization. After this
 * method has been executed the methods get_distances_data_size() and
 * get_echo_data_size() must return valid data.
 * @param config configuration
 * @param logger logger instance
 */

/** @var fawkes::Mutex * LaserAcquisitionThread::_data_mutex
 * Lock while writing to distances or echoes array or marking new data
 */

/** @var bool LaserAcquisitionThread::_new_data
 * Set to true in your loop if new data is available. Set to false automatically
 * in get_distance_data() and get_echoes_data().
 */

/** @var float * LaserAcquisitionThread::_distances
 * Allocate a float array and copy your distance values measured in meters here.
 */

/** @var float * LaserAcquisitionThread::_echoes
 * Allocate a float array and copy your echo values here.
 */

/** @var unsigned int LaserAcquisitionThread::_distances_size
 * Assign this the size of the _distances array
 */

/** @var unsigned int LaserAcquisitionThread::_echoes_size
 * Assign this the size of the _echoes array
 */

/** @var fawkes::Time * LaserAcquisitionThread::_timestamp
 * Time when the most recent data was received.
 */


/** Constructor.
 * @param thread_name name of the thread, be descriptive
 */
LaserAcquisitionThread::LaserAcquisitionThread(const char *thread_name)
  : Thread(thread_name, Thread::OPMODE_CONTINUOUS)
{
  _data_mutex = new Mutex();
  _timestamp  = new Time();
  _new_data   = false;
  _distances = NULL;
  _echoes = NULL;
  _distances_size = 0;
  _echoes_size = 0;
}

LaserAcquisitionThread::~LaserAcquisitionThread()
{
  delete _data_mutex;
  delete _timestamp;
}


/** Lock data if fresh.
 * If new data has been received since get_distance_data() or get_echo_data()
 * was called last the data is locked, no new data can arrive until you call
 * unlock(), otherwise the lock is immediately released after checking.
 * @return true if the lock was acquired and there is new data, false otherwise
 */
bool
LaserAcquisitionThread::lock_if_new_data()
{
  _data_mutex->lock();
  if (_new_data) {
    return true;
  } else {
    _data_mutex->unlock();
    return false;
  }
}


/** Unlock data, */
void
LaserAcquisitionThread::unlock()
{
  _data_mutex->unlock();
}


/** Get distance data.
 * @return Float array with distance values
 */
const float *
LaserAcquisitionThread::get_distance_data()
{
  _new_data = false;
  return _distances;
}


/** Get echo data.
 * @return Float array with echo values
 */
const float *
LaserAcquisitionThread::get_echo_data()
{
  _new_data = false;
  return _echoes;
}


/** Get distance data size.
 * @return size of data float array
 */
unsigned int
LaserAcquisitionThread::get_distance_data_size()
{
  return _distances_size;
}


/** Get echo data size.
 * @return size of data float array
 */
unsigned int
LaserAcquisitionThread::get_echo_data_size()
{
  return _echoes_size;
}


/** Get timestamp of data
 * @return most recent data time
 */
const fawkes::Time *
LaserAcquisitionThread::get_timestamp()
{
  return _timestamp;
}


/** Allocate distances array.
 * Call this from a laser acqusition thread implementation to properly
 * initialize the distances array.
 * @param num_distances number of distances to allocate the array for
 */
void
LaserAcquisitionThread::alloc_distances(unsigned int num_distances)
{
  if (_distances)  free(_distances);

  _distances_size = num_distances;
  _distances      = (float *)malloc(sizeof(float) * _distances_size);
  std::fill_n(_distances, _distances_size,
              std::numeric_limits<float>::quiet_NaN());
}


/** Allocate echoes array.
 * Call this from a laser acqusition thread implementation to properly
 * initialize the echoes array.
 * @param num_echoes number of echoes to allocate the array for
 */
void
LaserAcquisitionThread::alloc_echoes(unsigned int num_echoes)
{
  if (_echoes)  free(_echoes);

  _echoes_size = num_echoes;
  _echoes      = (float *)malloc(sizeof(float) * _echoes_size);
  memset(_echoes, 0, sizeof(float) * _echoes_size);
}


/** Reset all distance values to NaN. */
void
LaserAcquisitionThread::reset_distances()
{
  _data_mutex->lock();
  if (! _distances)  return;

  for (size_t i = 0; i < _distances_size; ++i) {
    _distances[i] = std::numeric_limits<float>::quiet_NaN();
  }
  _new_data = true;
  _data_mutex->unlock();
}


/** Reset all distance values to NaN. */
void
LaserAcquisitionThread::reset_echoes()
{
  if (! _echoes)  return;

  for (size_t i = 0; i < _echoes_size; ++i) {
    _echoes[i] = std::numeric_limits<float>::quiet_NaN();
  }
}
