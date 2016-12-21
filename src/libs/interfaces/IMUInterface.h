
/***************************************************************************
 *  IMUInterface.h - Fawkes BlackBoard Interface - IMUInterface
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

#ifndef __INTERFACES_IMUINTERFACE_H_
#define __INTERFACES_IMUINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class IMUInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(IMUInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char frame[32]; /**< 
      Coordinate frame in which the data is presented.
     */
    float orientation[4]; /**< 
      Rotation quaternion ordered as (x, y, z, w).
     */
    double orientation_covariance[9]; /**< 
      Covariance of orientation, row major about x, y, z axes.
     */
    float angular_velocity[3]; /**< 
      Angular velocities ordered as (x, y, z).
     */
    double angular_velocity_covariance[9]; /**< 
      Covariance of angular velocity, row major about x, y, z axes.
     */
    float linear_acceleration[3]; /**< 
      Linear acceleration ordered as (x, y, z).
     */
    double linear_acceleration_covariance[9]; /**< 
      Covariance of linear acceleration, row major about x, y, z axes.
     */
  } IMUInterface_data_t;

  IMUInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  IMUInterface();
  ~IMUInterface();

 public:
  /* Methods */
  char * frame() const;
  void set_frame(const char * new_frame);
  size_t maxlenof_frame() const;
  float * orientation() const;
  float orientation(unsigned int index) const;
  void set_orientation(unsigned int index, const float new_orientation);
  void set_orientation(const float * new_orientation);
  size_t maxlenof_orientation() const;
  double * orientation_covariance() const;
  double orientation_covariance(unsigned int index) const;
  void set_orientation_covariance(unsigned int index, const double new_orientation_covariance);
  void set_orientation_covariance(const double * new_orientation_covariance);
  size_t maxlenof_orientation_covariance() const;
  float * angular_velocity() const;
  float angular_velocity(unsigned int index) const;
  void set_angular_velocity(unsigned int index, const float new_angular_velocity);
  void set_angular_velocity(const float * new_angular_velocity);
  size_t maxlenof_angular_velocity() const;
  double * angular_velocity_covariance() const;
  double angular_velocity_covariance(unsigned int index) const;
  void set_angular_velocity_covariance(unsigned int index, const double new_angular_velocity_covariance);
  void set_angular_velocity_covariance(const double * new_angular_velocity_covariance);
  size_t maxlenof_angular_velocity_covariance() const;
  float * linear_acceleration() const;
  float linear_acceleration(unsigned int index) const;
  void set_linear_acceleration(unsigned int index, const float new_linear_acceleration);
  void set_linear_acceleration(const float * new_linear_acceleration);
  size_t maxlenof_linear_acceleration() const;
  double * linear_acceleration_covariance() const;
  double linear_acceleration_covariance(unsigned int index) const;
  void set_linear_acceleration_covariance(unsigned int index, const double new_linear_acceleration_covariance);
  void set_linear_acceleration_covariance(const double * new_linear_acceleration_covariance);
  size_t maxlenof_linear_acceleration_covariance() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
