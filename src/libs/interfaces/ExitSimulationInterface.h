
/***************************************************************************
 *  ExitSimulationInterface.h - Fawkes BlackBoard Interface - ExitSimulationInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Gesche Gierse
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

#ifndef __INTERFACES_EXITSIMULATIONINTERFACE_H_
#define __INTERFACES_EXITSIMULATIONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class ExitSimulationInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(ExitSimulationInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    bool shutdown_initiated; /**< Whether a shutdown was initiated */
  } ExitSimulationInterface_data_t;
#pragma pack(pop)

  ExitSimulationInterface_data_t *data;

 public:
  /* messages */
  class ExitSimulationMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } ExitSimulationMessage_data_t;
#pragma pack(pop)

    ExitSimulationMessage_data_t *data;

   public:
    ExitSimulationMessage();
    ~ExitSimulationMessage();

    ExitSimulationMessage(const ExitSimulationMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  ExitSimulationInterface();
  ~ExitSimulationInterface();

 public:
  /* Methods */
  bool is_shutdown_initiated() const;
  void set_shutdown_initiated(const bool new_shutdown_initiated);
  size_t maxlenof_shutdown_initiated() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
