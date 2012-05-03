
/***************************************************************************
 *  RobotinoSensorInterface.cpp - Fawkes BlackBoard Interface - RobotinoSensorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2012  Tim Niemueller
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

#include <interfaces/RobotinoSensorInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class RobotinoSensorInterface <interfaces/RobotinoSensorInterface.h>
 * RobotinoSensorInterface Fawkes BlackBoard Interface.
 * Sensor information of a Robotino robot
 * @ingroup FawkesInterfaces
 */



/** Constructor */
RobotinoSensorInterface::RobotinoSensorInterface() : Interface()
{
  data_size = sizeof(RobotinoSensorInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (RobotinoSensorInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_FLOAT, "mot_velocity", 3, &data->mot_velocity);
  add_fieldinfo(IFT_INT32, "mot_position", 3, &data->mot_position);
  add_fieldinfo(IFT_FLOAT, "mot_current", 3, &data->mot_current);
  add_fieldinfo(IFT_BOOL, "bumper", 1, &data->bumper);
  add_fieldinfo(IFT_FLOAT, "distance", 9, &data->distance);
  add_fieldinfo(IFT_BOOL, "digital_in", 8, &data->digital_in);
  add_fieldinfo(IFT_FLOAT, "analog_in", 8, &data->analog_in);
  add_fieldinfo(IFT_BOOL, "gyro_available", 1, &data->gyro_available);
  add_fieldinfo(IFT_FLOAT, "gyro_angle", 1, &data->gyro_angle);
  add_fieldinfo(IFT_FLOAT, "gyro_rate", 1, &data->gyro_rate);
  unsigned char tmp_hash[] = {0xfe, 0x8a, 0xcf, 0x8f, 0xe8, 0xb9, 0xf, 0x3b, 0x35, 0x3b, 0x1a, 0xea, 0xe7, 0x59, 0xab, 0xc8};
  set_hash(tmp_hash);
}

/** Destructor */
RobotinoSensorInterface::~RobotinoSensorInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get mot_velocity value.
 * Velocities of the wheels.
 * @return mot_velocity value
 */
float *
RobotinoSensorInterface::mot_velocity() const
{
  return data->mot_velocity;
}

/** Get mot_velocity value at given index.
 * Velocities of the wheels.
 * @param index index of value
 * @return mot_velocity value
 * @exception Exception thrown if index is out of bounds
 */
float
RobotinoSensorInterface::mot_velocity(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->mot_velocity[index];
}

/** Get maximum length of mot_velocity value.
 * @return length of mot_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoSensorInterface::maxlenof_mot_velocity() const
{
  return 3;
}

/** Set mot_velocity value.
 * Velocities of the wheels.
 * @param new_mot_velocity new mot_velocity value
 */
void
RobotinoSensorInterface::set_mot_velocity(const float * new_mot_velocity)
{
  memcpy(data->mot_velocity, new_mot_velocity, sizeof(float) * 3);
  data_changed = true;
}

/** Set mot_velocity value at given index.
 * Velocities of the wheels.
 * @param new_mot_velocity new mot_velocity value
 * @param index index for of the value
 */
void
RobotinoSensorInterface::set_mot_velocity(unsigned int index, const float new_mot_velocity)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->mot_velocity[index] = new_mot_velocity;
}
/** Get mot_position value.
 * Positions of the wheels.
 * @return mot_position value
 */
int32_t *
RobotinoSensorInterface::mot_position() const
{
  return data->mot_position;
}

/** Get mot_position value at given index.
 * Positions of the wheels.
 * @param index index of value
 * @return mot_position value
 * @exception Exception thrown if index is out of bounds
 */
int32_t
RobotinoSensorInterface::mot_position(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->mot_position[index];
}

/** Get maximum length of mot_position value.
 * @return length of mot_position value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoSensorInterface::maxlenof_mot_position() const
{
  return 3;
}

/** Set mot_position value.
 * Positions of the wheels.
 * @param new_mot_position new mot_position value
 */
void
RobotinoSensorInterface::set_mot_position(const int32_t * new_mot_position)
{
  memcpy(data->mot_position, new_mot_position, sizeof(int32_t) * 3);
  data_changed = true;
}

/** Set mot_position value at given index.
 * Positions of the wheels.
 * @param new_mot_position new mot_position value
 * @param index index for of the value
 */
void
RobotinoSensorInterface::set_mot_position(unsigned int index, const int32_t new_mot_position)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->mot_position[index] = new_mot_position;
}
/** Get mot_current value.
 * Motor currents.
 * @return mot_current value
 */
float *
RobotinoSensorInterface::mot_current() const
{
  return data->mot_current;
}

/** Get mot_current value at given index.
 * Motor currents.
 * @param index index of value
 * @return mot_current value
 * @exception Exception thrown if index is out of bounds
 */
float
RobotinoSensorInterface::mot_current(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->mot_current[index];
}

/** Get maximum length of mot_current value.
 * @return length of mot_current value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoSensorInterface::maxlenof_mot_current() const
{
  return 3;
}

/** Set mot_current value.
 * Motor currents.
 * @param new_mot_current new mot_current value
 */
void
RobotinoSensorInterface::set_mot_current(const float * new_mot_current)
{
  memcpy(data->mot_current, new_mot_current, sizeof(float) * 3);
  data_changed = true;
}

/** Set mot_current value at given index.
 * Motor currents.
 * @param new_mot_current new mot_current value
 * @param index index for of the value
 */
void
RobotinoSensorInterface::set_mot_current(unsigned int index, const float new_mot_current)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->mot_current[index] = new_mot_current;
}
/** Get bumper value.
 * Bumper pressed indicator.
 * @return bumper value
 */
bool
RobotinoSensorInterface::is_bumper() const
{
  return data->bumper;
}

/** Get maximum length of bumper value.
 * @return length of bumper value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoSensorInterface::maxlenof_bumper() const
{
  return 1;
}

/** Set bumper value.
 * Bumper pressed indicator.
 * @param new_bumper new bumper value
 */
void
RobotinoSensorInterface::set_bumper(const bool new_bumper)
{
  data->bumper = new_bumper;
  data_changed = true;
}

/** Get distance value.
 * Distance sensor values.
 * @return distance value
 */
float *
RobotinoSensorInterface::distance() const
{
  return data->distance;
}

/** Get distance value at given index.
 * Distance sensor values.
 * @param index index of value
 * @return distance value
 * @exception Exception thrown if index is out of bounds
 */
float
RobotinoSensorInterface::distance(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->distance[index];
}

/** Get maximum length of distance value.
 * @return length of distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoSensorInterface::maxlenof_distance() const
{
  return 9;
}

/** Set distance value.
 * Distance sensor values.
 * @param new_distance new distance value
 */
void
RobotinoSensorInterface::set_distance(const float * new_distance)
{
  memcpy(data->distance, new_distance, sizeof(float) * 9);
  data_changed = true;
}

/** Set distance value at given index.
 * Distance sensor values.
 * @param new_distance new distance value
 * @param index index for of the value
 */
void
RobotinoSensorInterface::set_distance(unsigned int index, const float new_distance)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->distance[index] = new_distance;
}
/** Get digital_in value.
 * Digital input values.
 * @return digital_in value
 */
bool *
RobotinoSensorInterface::is_digital_in() const
{
  return data->digital_in;
}

/** Get digital_in value at given index.
 * Digital input values.
 * @param index index of value
 * @return digital_in value
 * @exception Exception thrown if index is out of bounds
 */
bool
RobotinoSensorInterface::is_digital_in(unsigned int index) const
{
  if (index > 8) {
    throw Exception("Index value %u out of bounds (0..8)", index);
  }
  return data->digital_in[index];
}

/** Get maximum length of digital_in value.
 * @return length of digital_in value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoSensorInterface::maxlenof_digital_in() const
{
  return 8;
}

/** Set digital_in value.
 * Digital input values.
 * @param new_digital_in new digital_in value
 */
void
RobotinoSensorInterface::set_digital_in(const bool * new_digital_in)
{
  memcpy(data->digital_in, new_digital_in, sizeof(bool) * 8);
  data_changed = true;
}

/** Set digital_in value at given index.
 * Digital input values.
 * @param new_digital_in new digital_in value
 * @param index index for of the value
 */
void
RobotinoSensorInterface::set_digital_in(unsigned int index, const bool new_digital_in)
{
  if (index > 8) {
    throw Exception("Index value %u out of bounds (0..8)", index);
  }
  data->digital_in[index] = new_digital_in;
}
/** Get analog_in value.
 * Analog input values.
 * @return analog_in value
 */
float *
RobotinoSensorInterface::analog_in() const
{
  return data->analog_in;
}

/** Get analog_in value at given index.
 * Analog input values.
 * @param index index of value
 * @return analog_in value
 * @exception Exception thrown if index is out of bounds
 */
float
RobotinoSensorInterface::analog_in(unsigned int index) const
{
  if (index > 8) {
    throw Exception("Index value %u out of bounds (0..8)", index);
  }
  return data->analog_in[index];
}

/** Get maximum length of analog_in value.
 * @return length of analog_in value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoSensorInterface::maxlenof_analog_in() const
{
  return 8;
}

/** Set analog_in value.
 * Analog input values.
 * @param new_analog_in new analog_in value
 */
void
RobotinoSensorInterface::set_analog_in(const float * new_analog_in)
{
  memcpy(data->analog_in, new_analog_in, sizeof(float) * 8);
  data_changed = true;
}

/** Set analog_in value at given index.
 * Analog input values.
 * @param new_analog_in new analog_in value
 * @param index index for of the value
 */
void
RobotinoSensorInterface::set_analog_in(unsigned int index, const float new_analog_in)
{
  if (index > 8) {
    throw Exception("Index value %u out of bounds (0..8)", index);
  }
  data->analog_in[index] = new_analog_in;
}
/** Get gyro_available value.
 * True if gyro is available
 * @return gyro_available value
 */
bool
RobotinoSensorInterface::is_gyro_available() const
{
  return data->gyro_available;
}

/** Get maximum length of gyro_available value.
 * @return length of gyro_available value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoSensorInterface::maxlenof_gyro_available() const
{
  return 1;
}

/** Set gyro_available value.
 * True if gyro is available
 * @param new_gyro_available new gyro_available value
 */
void
RobotinoSensorInterface::set_gyro_available(const bool new_gyro_available)
{
  data->gyro_available = new_gyro_available;
  data_changed = true;
}

/** Get gyro_angle value.
 * Gyro angle value; rad
 * @return gyro_angle value
 */
float
RobotinoSensorInterface::gyro_angle() const
{
  return data->gyro_angle;
}

/** Get maximum length of gyro_angle value.
 * @return length of gyro_angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoSensorInterface::maxlenof_gyro_angle() const
{
  return 1;
}

/** Set gyro_angle value.
 * Gyro angle value; rad
 * @param new_gyro_angle new gyro_angle value
 */
void
RobotinoSensorInterface::set_gyro_angle(const float new_gyro_angle)
{
  data->gyro_angle = new_gyro_angle;
  data_changed = true;
}

/** Get gyro_rate value.
 * Gyro rate value; rad/sec
 * @return gyro_rate value
 */
float
RobotinoSensorInterface::gyro_rate() const
{
  return data->gyro_rate;
}

/** Get maximum length of gyro_rate value.
 * @return length of gyro_rate value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoSensorInterface::maxlenof_gyro_rate() const
{
  return 1;
}

/** Set gyro_rate value.
 * Gyro rate value; rad/sec
 * @param new_gyro_rate new gyro_rate value
 */
void
RobotinoSensorInterface::set_gyro_rate(const float new_gyro_rate)
{
  data->gyro_rate = new_gyro_rate;
  data_changed = true;
}

/* =========== message create =========== */
Message *
RobotinoSensorInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
RobotinoSensorInterface::copy_values(const Interface *other)
{
  const RobotinoSensorInterface *oi = dynamic_cast<const RobotinoSensorInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(RobotinoSensorInterface_data_t));
}

const char *
RobotinoSensorInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
RobotinoSensorInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(RobotinoSensorInterface)
/// @endcond


} // end namespace fawkes
