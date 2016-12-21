
/***************************************************************************
 *  LocalizationInterface.h - Fawkes BlackBoard Interface - LocalizationInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2015  Tim Niemueller
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

#ifndef __INTERFACES_LOCALIZATIONINTERFACE_H_
#define __INTERFACES_LOCALIZATIONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class LocalizationInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(LocalizationInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char map[64]; /**< The currently used map. */
  } LocalizationInterface_data_t;

  LocalizationInterface_data_t *data;

 public:
  /* messages */
  class SetInitialPoseMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char frame[32]; /**< 
      Reference coordinate frame for the data.
     */
      double rotation[4]; /**< 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
     */
      double translation[3]; /**< 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
     */
      double covariance[36]; /**< 
      Row-major representation of the 6x6 covariance matrix. The
      orientation parameters use a fixed-axis representation. In
      order, the parameters are: (x, y, z, rotation about X axis,
      rotation about Y axis, rotation about Z axis).
     */
    } SetInitialPoseMessage_data_t;

    SetInitialPoseMessage_data_t *data;

   public:
    SetInitialPoseMessage(const char * ini_frame, const double * ini_rotation, const double * ini_translation, const double * ini_covariance);
    SetInitialPoseMessage();
    ~SetInitialPoseMessage();

    SetInitialPoseMessage(const SetInitialPoseMessage *m);
    /* Methods */
    char * frame() const;
    void set_frame(const char * new_frame);
    size_t maxlenof_frame() const;
    double * rotation() const;
    double rotation(unsigned int index) const;
    void set_rotation(unsigned int index, const double new_rotation);
    void set_rotation(const double * new_rotation);
    size_t maxlenof_rotation() const;
    double * translation() const;
    double translation(unsigned int index) const;
    void set_translation(unsigned int index, const double new_translation);
    void set_translation(const double * new_translation);
    size_t maxlenof_translation() const;
    double * covariance() const;
    double covariance(unsigned int index) const;
    void set_covariance(unsigned int index, const double new_covariance);
    void set_covariance(const double * new_covariance);
    size_t maxlenof_covariance() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  LocalizationInterface();
  ~LocalizationInterface();

 public:
  /* Methods */
  char * map() const;
  void set_map(const char * new_map);
  size_t maxlenof_map() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
