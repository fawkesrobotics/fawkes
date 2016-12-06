
/***************************************************************************
 *  PddlGenInterface.h - Fawkes BlackBoard Interface - PddlGenInterface
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

#ifndef __INTERFACES_PDDLGENINTERFACE_H_
#define __INTERFACES_PDDLGENINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class PddlGenInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(PddlGenInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint32_t msg_id; /**< 
      The ID of the message that was processed last
     */
    bool final; /**< 
      Is the generation finished?
     */
  } PddlGenInterface_data_t;
#pragma pack(pop)

  PddlGenInterface_data_t *data;

 public:
  /* messages */
  class GenerateMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char goal[1024]; /**< 
    	Optional goal to insert into the template dictionary.
     */
    } GenerateMessage_data_t;
#pragma pack(pop)

    GenerateMessage_data_t *data;

   public:
    GenerateMessage(const char * ini_goal);
    GenerateMessage();
    ~GenerateMessage();

    GenerateMessage(const GenerateMessage *m);
    /* Methods */
    char * goal() const;
    void set_goal(const char * new_goal);
    size_t maxlenof_goal() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  PddlGenInterface();
  ~PddlGenInterface();

 public:
  /* Methods */
  uint32_t msg_id() const;
  void set_msg_id(const uint32_t new_msg_id);
  size_t maxlenof_msg_id() const;
  bool is_final() const;
  void set_final(const bool new_final);
  size_t maxlenof_final() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
