
/***************************************************************************
 *  TransformInterface.h - Fawkes BlackBoard Interface - TransformInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2011  Tim Niemueller
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

#ifndef __INTERFACES_TRANSFORMINTERFACE_H_
#define __INTERFACES_TRANSFORMINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class TransformInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(TransformInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char frame[64]; /**< 
      Parent frame ID. The given transform is relative to the origin
      of this coordinate frame.
     */
    char child_frame[64]; /**< 
      The ID of the child frame. The child frame's origin is at the
      given point in the parent frame denoted by the transform.
     */
    bool static_transform; /**< 
	    True if the transform is static, i.e. it will never change
	    during its lifetime, false otherwise.
     */
    double translation[3]; /**< 
      This array denotes the translation vector of the transform. The
      element indexes are ordered x, y, z, i.e. translation[0] is the
      X value of the translation vector.
     */
    double rotation[4]; /**< 
      This array denotes the rotation quaternion of the transform. The
      element indexes are ordered x, y, z, w, i.e. translation[0] is
      the X value of the rotation quaternion and translation[3] is the
      W value.
     */
  } TransformInterface_data_t;
#pragma pack(pop)

  TransformInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  TransformInterface();
  ~TransformInterface();

 public:
  /* Methods */
  char * frame() const;
  void set_frame(const char * new_frame);
  size_t maxlenof_frame() const;
  char * child_frame() const;
  void set_child_frame(const char * new_child_frame);
  size_t maxlenof_child_frame() const;
  bool is_static_transform() const;
  void set_static_transform(const bool new_static_transform);
  size_t maxlenof_static_transform() const;
  double * translation() const;
  double translation(unsigned int index) const;
  void set_translation(unsigned int index, const double new_translation);
  void set_translation(const double * new_translation);
  size_t maxlenof_translation() const;
  double * rotation() const;
  double rotation(unsigned int index) const;
  void set_rotation(unsigned int index, const double new_rotation);
  void set_rotation(const double * new_rotation);
  size_t maxlenof_rotation() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
