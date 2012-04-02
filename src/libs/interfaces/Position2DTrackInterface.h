
/***************************************************************************
 *  Position2DTrackInterface.h - Fawkes BlackBoard Interface - Position2DTrackInterface
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

#ifndef __INTERFACES_POSITION2DTRACKINTERFACE_H_
#define __INTERFACES_POSITION2DTRACKINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class Position2DTrackInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(Position2DTrackInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    float track_x_positions[30]; /**< 
      X-Positions of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
     */
    float track_y_positions[30]; /**< 
      Y-Positions of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
     */
    int32_t track_timestamps[30]; /**< 
      Timestamps of the track. The first array-element is the oldest position of the track, 
      the last is the newest.
     */
    bool valid; /**< True, if this track is valid. */
    uint32_t length; /**< Length of the Tracks (i.e. up to which index there are valid positions). */
    uint32_t track_id; /**< The ID of the Track. */
  } Position2DTrackInterface_data_t;
#pragma pack(pop)

  Position2DTrackInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  Position2DTrackInterface();
  ~Position2DTrackInterface();

 public:
  /* Methods */
  float * track_x_positions() const;
  float track_x_positions(unsigned int index) const;
  void set_track_x_positions(unsigned int index, const float new_track_x_positions);
  void set_track_x_positions(const float * new_track_x_positions);
  size_t maxlenof_track_x_positions() const;
  float * track_y_positions() const;
  float track_y_positions(unsigned int index) const;
  void set_track_y_positions(unsigned int index, const float new_track_y_positions);
  void set_track_y_positions(const float * new_track_y_positions);
  size_t maxlenof_track_y_positions() const;
  int32_t * track_timestamps() const;
  int32_t track_timestamps(unsigned int index) const;
  void set_track_timestamps(unsigned int index, const int32_t new_track_timestamps);
  void set_track_timestamps(const int32_t * new_track_timestamps);
  size_t maxlenof_track_timestamps() const;
  bool is_valid() const;
  void set_valid(const bool new_valid);
  size_t maxlenof_valid() const;
  uint32_t length() const;
  void set_length(const uint32_t new_length);
  size_t maxlenof_length() const;
  uint32_t track_id() const;
  void set_track_id(const uint32_t new_track_id);
  size_t maxlenof_track_id() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
