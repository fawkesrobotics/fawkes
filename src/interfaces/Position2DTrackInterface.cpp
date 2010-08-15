
/***************************************************************************
 *  Position2DTrackInterface.cpp - Fawkes BlackBoard Interface - Position2DTrackInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2009  Masrur Doostdar
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

#include <interfaces/Position2DTrackInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class Position2DTrackInterface <interfaces/Position2DTrackInterface.h>
 * Position2DTrackInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to a track of 2D positions.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
Position2DTrackInterface::Position2DTrackInterface() : Interface()
{
  data_size = sizeof(Position2DTrackInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (Position2DTrackInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_FLOAT, "track_x_positions", 30, &data->track_x_positions);
  add_fieldinfo(IFT_FLOAT, "track_y_positions", 30, &data->track_y_positions);
  add_fieldinfo(IFT_INT32, "track_timestamps", 30, &data->track_timestamps);
  add_fieldinfo(IFT_BOOL, "valid", 1, &data->valid);
  add_fieldinfo(IFT_UINT32, "length", 1, &data->length);
  add_fieldinfo(IFT_UINT32, "track_id", 1, &data->track_id);
  unsigned char tmp_hash[] = {0xcd, 0xb8, 0x68, 0x14, 0xff, 0x3, 0xe4, 0xc4, 0x20, 0x43, 0x44, 0xb8, 0x86, 0x87, 0xa3, 0x4c};
  set_hash(tmp_hash);
}

/** Destructor */
Position2DTrackInterface::~Position2DTrackInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get track_x_positions value.
 * 
      X-Positions of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @return track_x_positions value
 */
float *
Position2DTrackInterface::track_x_positions() const
{
  return data->track_x_positions;
}

/** Get track_x_positions value at given index.
 * 
      X-Positions of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @param index index of value
 * @return track_x_positions value
 * @exception Exception thrown if index is out of bounds
 */
float
Position2DTrackInterface::track_x_positions(unsigned int index) const
{
  if (index > 30) {
    throw Exception("Index value %u out of bounds (0..30)", index);
  }
  return data->track_x_positions[index];
}

/** Get maximum length of track_x_positions value.
 * @return length of track_x_positions value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position2DTrackInterface::maxlenof_track_x_positions() const
{
  return 30;
}

/** Set track_x_positions value.
 * 
      X-Positions of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @param new_track_x_positions new track_x_positions value
 */
void
Position2DTrackInterface::set_track_x_positions(const float * new_track_x_positions)
{
  memcpy(data->track_x_positions, new_track_x_positions, sizeof(float) * 30);
  data_changed = true;
}

/** Set track_x_positions value at given index.
 * 
      X-Positions of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @param new_track_x_positions new track_x_positions value
 * @param index index for of the value
 */
void
Position2DTrackInterface::set_track_x_positions(unsigned int index, const float new_track_x_positions)
{
  if (index > 30) {
    throw Exception("Index value %u out of bounds (0..30)", index);
  }
  data->track_x_positions[index] = new_track_x_positions;
}
/** Get track_y_positions value.
 * 
      Y-Positions of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @return track_y_positions value
 */
float *
Position2DTrackInterface::track_y_positions() const
{
  return data->track_y_positions;
}

/** Get track_y_positions value at given index.
 * 
      Y-Positions of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @param index index of value
 * @return track_y_positions value
 * @exception Exception thrown if index is out of bounds
 */
float
Position2DTrackInterface::track_y_positions(unsigned int index) const
{
  if (index > 30) {
    throw Exception("Index value %u out of bounds (0..30)", index);
  }
  return data->track_y_positions[index];
}

/** Get maximum length of track_y_positions value.
 * @return length of track_y_positions value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position2DTrackInterface::maxlenof_track_y_positions() const
{
  return 30;
}

/** Set track_y_positions value.
 * 
      Y-Positions of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @param new_track_y_positions new track_y_positions value
 */
void
Position2DTrackInterface::set_track_y_positions(const float * new_track_y_positions)
{
  memcpy(data->track_y_positions, new_track_y_positions, sizeof(float) * 30);
  data_changed = true;
}

/** Set track_y_positions value at given index.
 * 
      Y-Positions of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @param new_track_y_positions new track_y_positions value
 * @param index index for of the value
 */
void
Position2DTrackInterface::set_track_y_positions(unsigned int index, const float new_track_y_positions)
{
  if (index > 30) {
    throw Exception("Index value %u out of bounds (0..30)", index);
  }
  data->track_y_positions[index] = new_track_y_positions;
}
/** Get track_timestamps value.
 * 
      Timestamps of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @return track_timestamps value
 */
int32_t *
Position2DTrackInterface::track_timestamps() const
{
  return data->track_timestamps;
}

/** Get track_timestamps value at given index.
 * 
      Timestamps of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @param index index of value
 * @return track_timestamps value
 * @exception Exception thrown if index is out of bounds
 */
int32_t
Position2DTrackInterface::track_timestamps(unsigned int index) const
{
  if (index > 30) {
    throw Exception("Index value %u out of bounds (0..30)", index);
  }
  return data->track_timestamps[index];
}

/** Get maximum length of track_timestamps value.
 * @return length of track_timestamps value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position2DTrackInterface::maxlenof_track_timestamps() const
{
  return 30;
}

/** Set track_timestamps value.
 * 
      Timestamps of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @param new_track_timestamps new track_timestamps value
 */
void
Position2DTrackInterface::set_track_timestamps(const int32_t * new_track_timestamps)
{
  memcpy(data->track_timestamps, new_track_timestamps, sizeof(int32_t) * 30);
  data_changed = true;
}

/** Set track_timestamps value at given index.
 * 
      Timestamps of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
    
 * @param new_track_timestamps new track_timestamps value
 * @param index index for of the value
 */
void
Position2DTrackInterface::set_track_timestamps(unsigned int index, const int32_t new_track_timestamps)
{
  if (index > 30) {
    throw Exception("Index value %u out of bounds (0..30)", index);
  }
  data->track_timestamps[index] = new_track_timestamps;
}
/** Get valid value.
 * True, if this track is valid.
 * @return valid value
 */
bool
Position2DTrackInterface::is_valid() const
{
  return data->valid;
}

/** Get maximum length of valid value.
 * @return length of valid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position2DTrackInterface::maxlenof_valid() const
{
  return 1;
}

/** Set valid value.
 * True, if this track is valid.
 * @param new_valid new valid value
 */
void
Position2DTrackInterface::set_valid(const bool new_valid)
{
  data->valid = new_valid;
  data_changed = true;
}

/** Get length value.
 * Length of the Tracks (i.e. up to which index there are valid positions).
 * @return length value
 */
uint32_t
Position2DTrackInterface::length() const
{
  return data->length;
}

/** Get maximum length of length value.
 * @return length of length value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position2DTrackInterface::maxlenof_length() const
{
  return 1;
}

/** Set length value.
 * Length of the Tracks (i.e. up to which index there are valid positions).
 * @param new_length new length value
 */
void
Position2DTrackInterface::set_length(const uint32_t new_length)
{
  data->length = new_length;
  data_changed = true;
}

/** Get track_id value.
 * The ID of the Track.
 * @return track_id value
 */
uint32_t
Position2DTrackInterface::track_id() const
{
  return data->track_id;
}

/** Get maximum length of track_id value.
 * @return length of track_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position2DTrackInterface::maxlenof_track_id() const
{
  return 1;
}

/** Set track_id value.
 * The ID of the Track.
 * @param new_track_id new track_id value
 */
void
Position2DTrackInterface::set_track_id(const uint32_t new_track_id)
{
  data->track_id = new_track_id;
  data_changed = true;
}

/* =========== message create =========== */
Message *
Position2DTrackInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
Position2DTrackInterface::copy_values(const Interface *other)
{
  const Position2DTrackInterface *oi = dynamic_cast<const Position2DTrackInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(Position2DTrackInterface_data_t));
}

const char *
Position2DTrackInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
Position2DTrackInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(Position2DTrackInterface)
/// @endcond


} // end namespace fawkes
