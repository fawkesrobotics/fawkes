
/***************************************************************************
 *  OpenraveRobotMemoryInterface.h - Fawkes BlackBoard Interface - OpenraveRobotMemoryInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Frederik Zwilling
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

#ifndef __INTERFACES_OPENRAVEROBOTMEMORYINTERFACE_H_
#define __INTERFACES_OPENRAVEROBOTMEMORYINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class OpenraveRobotMemoryInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(OpenraveRobotMemoryInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint32_t dummy; /**< 
      Dummy field
     */
  } OpenraveRobotMemoryInterface_data_t;
#pragma pack(pop)

  OpenraveRobotMemoryInterface_data_t *data;

 public:
  /* messages */
  class ConstructSceneMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } ConstructSceneMessage_data_t;
#pragma pack(pop)

    ConstructSceneMessage_data_t *data;

   public:
    ConstructSceneMessage();
    ~ConstructSceneMessage();

    ConstructSceneMessage(const ConstructSceneMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  OpenraveRobotMemoryInterface();
  ~OpenraveRobotMemoryInterface();

 public:
  /* Methods */
  uint32_t dummy() const;
  void set_dummy(const uint32_t new_dummy);
  size_t maxlenof_dummy() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
