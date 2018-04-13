
/***************************************************************************
 *  IMUInterface.cpp - Fawkes BlackBoard Interface - IMUInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2014  Tim Niemueller
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

#include <interfaces/IMUInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class IMUInterface <interfaces/IMUInterface.h>
 * IMUInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to data of inertial measurement
      units. It is based on the sensor_msgs/Imu data type for
      compatibility.

      Accelerations should be in m/s^2 (not in g's), and rotational
      velocity should be in rad/sec.

      If the covariance of the measurement is known, it should be
      filled in (if all you know is the variance of each measurement,
      e.g. from the datasheet, just put those along the diagonal). A
      covariance matrix of all zeros will be interpreted as
      "covariance unknown", and to use the data a covariance will have
      to be assumed or gotten from some other source.

      If you have no estimate for one of the data elements (e.g. your
      IMU doesn't produce an orientation # estimate), please set
      element 0 of the associated covariance matrix to -1. If you are
      interpreting this message, please check for a value of -1 in the
      first element of each covariance matrix, and disregard the
      associated estimate.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
IMUInterface::IMUInterface() : Interface()
{
  data_size = sizeof(IMUInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (IMUInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_FLOAT, "orientation", 4, &data->orientation);
  add_fieldinfo(IFT_DOUBLE, "orientation_covariance", 9, &data->orientation_covariance);
  add_fieldinfo(IFT_FLOAT, "angular_velocity", 3, &data->angular_velocity);
  add_fieldinfo(IFT_DOUBLE, "angular_velocity_covariance", 9, &data->angular_velocity_covariance);
  add_fieldinfo(IFT_FLOAT, "linear_acceleration", 3, &data->linear_acceleration);
  add_fieldinfo(IFT_DOUBLE, "linear_acceleration_covariance", 9, &data->linear_acceleration_covariance);
  unsigned char tmp_hash[] = {0x9d, 0xf6, 0xde, 0x9d, 0x32, 0xe3, 0xf, 0x11, 0xac, 0xdc, 0x5d, 0x92, 0x27, 0x89, 0x27, 0x7e};
  set_hash(tmp_hash);
}

/** Destructor */
IMUInterface::~IMUInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get frame value.
 * 
      Coordinate frame in which the data is presented.
    
 * @return frame value
 */
char *
IMUInterface::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
IMUInterface::maxlenof_frame() const
{
  return 32;
}

/** Set frame value.
 * 
      Coordinate frame in which the data is presented.
    
 * @param new_frame new frame value
 */
void
IMUInterface::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame)-1);
  data->frame[sizeof(data->frame)-1] = 0;
  data_changed = true;
}

/** Get orientation value.
 * 
      Rotation quaternion ordered as (x, y, z, w).
    
 * @return orientation value
 */
float *
IMUInterface::orientation() const
{
  return data->orientation;
}

/** Get orientation value at given index.
 * 
      Rotation quaternion ordered as (x, y, z, w).
    
 * @param index index of value
 * @return orientation value
 * @exception Exception thrown if index is out of bounds
 */
float
IMUInterface::orientation(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->orientation[index];
}

/** Get maximum length of orientation value.
 * @return length of orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
IMUInterface::maxlenof_orientation() const
{
  return 4;
}

/** Set orientation value.
 * 
      Rotation quaternion ordered as (x, y, z, w).
    
 * @param new_orientation new orientation value
 */
void
IMUInterface::set_orientation(const float * new_orientation)
{
  memcpy(data->orientation, new_orientation, sizeof(float) * 4);
  data_changed = true;
}

/** Set orientation value at given index.
 * 
      Rotation quaternion ordered as (x, y, z, w).
    
 * @param new_orientation new orientation value
 * @param index index for of the value
 */
void
IMUInterface::set_orientation(unsigned int index, const float new_orientation)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->orientation[index] = new_orientation;
  data_changed = true;
}
/** Get orientation_covariance value.
 * 
      Covariance of orientation, row major about x, y, z axes.
    
 * @return orientation_covariance value
 */
double *
IMUInterface::orientation_covariance() const
{
  return data->orientation_covariance;
}

/** Get orientation_covariance value at given index.
 * 
      Covariance of orientation, row major about x, y, z axes.
    
 * @param index index of value
 * @return orientation_covariance value
 * @exception Exception thrown if index is out of bounds
 */
double
IMUInterface::orientation_covariance(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->orientation_covariance[index];
}

/** Get maximum length of orientation_covariance value.
 * @return length of orientation_covariance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
IMUInterface::maxlenof_orientation_covariance() const
{
  return 9;
}

/** Set orientation_covariance value.
 * 
      Covariance of orientation, row major about x, y, z axes.
    
 * @param new_orientation_covariance new orientation_covariance value
 */
void
IMUInterface::set_orientation_covariance(const double * new_orientation_covariance)
{
  memcpy(data->orientation_covariance, new_orientation_covariance, sizeof(double) * 9);
  data_changed = true;
}

/** Set orientation_covariance value at given index.
 * 
      Covariance of orientation, row major about x, y, z axes.
    
 * @param new_orientation_covariance new orientation_covariance value
 * @param index index for of the value
 */
void
IMUInterface::set_orientation_covariance(unsigned int index, const double new_orientation_covariance)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->orientation_covariance[index] = new_orientation_covariance;
  data_changed = true;
}
/** Get angular_velocity value.
 * 
      Angular velocities ordered as (x, y, z).
    
 * @return angular_velocity value
 */
float *
IMUInterface::angular_velocity() const
{
  return data->angular_velocity;
}

/** Get angular_velocity value at given index.
 * 
      Angular velocities ordered as (x, y, z).
    
 * @param index index of value
 * @return angular_velocity value
 * @exception Exception thrown if index is out of bounds
 */
float
IMUInterface::angular_velocity(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->angular_velocity[index];
}

/** Get maximum length of angular_velocity value.
 * @return length of angular_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
IMUInterface::maxlenof_angular_velocity() const
{
  return 3;
}

/** Set angular_velocity value.
 * 
      Angular velocities ordered as (x, y, z).
    
 * @param new_angular_velocity new angular_velocity value
 */
void
IMUInterface::set_angular_velocity(const float * new_angular_velocity)
{
  memcpy(data->angular_velocity, new_angular_velocity, sizeof(float) * 3);
  data_changed = true;
}

/** Set angular_velocity value at given index.
 * 
      Angular velocities ordered as (x, y, z).
    
 * @param new_angular_velocity new angular_velocity value
 * @param index index for of the value
 */
void
IMUInterface::set_angular_velocity(unsigned int index, const float new_angular_velocity)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->angular_velocity[index] = new_angular_velocity;
  data_changed = true;
}
/** Get angular_velocity_covariance value.
 * 
      Covariance of angular velocity, row major about x, y, z axes.
    
 * @return angular_velocity_covariance value
 */
double *
IMUInterface::angular_velocity_covariance() const
{
  return data->angular_velocity_covariance;
}

/** Get angular_velocity_covariance value at given index.
 * 
      Covariance of angular velocity, row major about x, y, z axes.
    
 * @param index index of value
 * @return angular_velocity_covariance value
 * @exception Exception thrown if index is out of bounds
 */
double
IMUInterface::angular_velocity_covariance(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->angular_velocity_covariance[index];
}

/** Get maximum length of angular_velocity_covariance value.
 * @return length of angular_velocity_covariance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
IMUInterface::maxlenof_angular_velocity_covariance() const
{
  return 9;
}

/** Set angular_velocity_covariance value.
 * 
      Covariance of angular velocity, row major about x, y, z axes.
    
 * @param new_angular_velocity_covariance new angular_velocity_covariance value
 */
void
IMUInterface::set_angular_velocity_covariance(const double * new_angular_velocity_covariance)
{
  memcpy(data->angular_velocity_covariance, new_angular_velocity_covariance, sizeof(double) * 9);
  data_changed = true;
}

/** Set angular_velocity_covariance value at given index.
 * 
      Covariance of angular velocity, row major about x, y, z axes.
    
 * @param new_angular_velocity_covariance new angular_velocity_covariance value
 * @param index index for of the value
 */
void
IMUInterface::set_angular_velocity_covariance(unsigned int index, const double new_angular_velocity_covariance)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->angular_velocity_covariance[index] = new_angular_velocity_covariance;
  data_changed = true;
}
/** Get linear_acceleration value.
 * 
      Linear acceleration ordered as (x, y, z).
    
 * @return linear_acceleration value
 */
float *
IMUInterface::linear_acceleration() const
{
  return data->linear_acceleration;
}

/** Get linear_acceleration value at given index.
 * 
      Linear acceleration ordered as (x, y, z).
    
 * @param index index of value
 * @return linear_acceleration value
 * @exception Exception thrown if index is out of bounds
 */
float
IMUInterface::linear_acceleration(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->linear_acceleration[index];
}

/** Get maximum length of linear_acceleration value.
 * @return length of linear_acceleration value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
IMUInterface::maxlenof_linear_acceleration() const
{
  return 3;
}

/** Set linear_acceleration value.
 * 
      Linear acceleration ordered as (x, y, z).
    
 * @param new_linear_acceleration new linear_acceleration value
 */
void
IMUInterface::set_linear_acceleration(const float * new_linear_acceleration)
{
  memcpy(data->linear_acceleration, new_linear_acceleration, sizeof(float) * 3);
  data_changed = true;
}

/** Set linear_acceleration value at given index.
 * 
      Linear acceleration ordered as (x, y, z).
    
 * @param new_linear_acceleration new linear_acceleration value
 * @param index index for of the value
 */
void
IMUInterface::set_linear_acceleration(unsigned int index, const float new_linear_acceleration)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->linear_acceleration[index] = new_linear_acceleration;
  data_changed = true;
}
/** Get linear_acceleration_covariance value.
 * 
      Covariance of linear acceleration, row major about x, y, z axes.
    
 * @return linear_acceleration_covariance value
 */
double *
IMUInterface::linear_acceleration_covariance() const
{
  return data->linear_acceleration_covariance;
}

/** Get linear_acceleration_covariance value at given index.
 * 
      Covariance of linear acceleration, row major about x, y, z axes.
    
 * @param index index of value
 * @return linear_acceleration_covariance value
 * @exception Exception thrown if index is out of bounds
 */
double
IMUInterface::linear_acceleration_covariance(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->linear_acceleration_covariance[index];
}

/** Get maximum length of linear_acceleration_covariance value.
 * @return length of linear_acceleration_covariance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
IMUInterface::maxlenof_linear_acceleration_covariance() const
{
  return 9;
}

/** Set linear_acceleration_covariance value.
 * 
      Covariance of linear acceleration, row major about x, y, z axes.
    
 * @param new_linear_acceleration_covariance new linear_acceleration_covariance value
 */
void
IMUInterface::set_linear_acceleration_covariance(const double * new_linear_acceleration_covariance)
{
  memcpy(data->linear_acceleration_covariance, new_linear_acceleration_covariance, sizeof(double) * 9);
  data_changed = true;
}

/** Set linear_acceleration_covariance value at given index.
 * 
      Covariance of linear acceleration, row major about x, y, z axes.
    
 * @param new_linear_acceleration_covariance new linear_acceleration_covariance value
 * @param index index for of the value
 */
void
IMUInterface::set_linear_acceleration_covariance(unsigned int index, const double new_linear_acceleration_covariance)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->linear_acceleration_covariance[index] = new_linear_acceleration_covariance;
  data_changed = true;
}
/* =========== message create =========== */
Message *
IMUInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
IMUInterface::copy_values(const Interface *other)
{
  const IMUInterface *oi = dynamic_cast<const IMUInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(IMUInterface_data_t));
}

const char *
IMUInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
IMUInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(IMUInterface)
/// @endcond


} // end namespace fawkes
