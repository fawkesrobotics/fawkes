
/***************************************************************************
 *  EclipseDebuggerInterface.h - Fawkes BlackBoard Interface - EclipseDebuggerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2012  Gesche Gierse
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

#ifndef __INTERFACES_ECLIPSEDEBUGGERINTERFACE_H_
#define __INTERFACES_ECLIPSEDEBUGGERINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class EclipseDebuggerInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(EclipseDebuggerInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint16_t port; /**< Port where to connect to */
    char host[100]; /**< Host where to connect to */
  } EclipseDebuggerInterface_data_t;

  EclipseDebuggerInterface_data_t *data;

 public:
  /* messages */
  class ConnectionMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } ConnectionMessage_data_t;

    ConnectionMessage_data_t *data;

   public:
    ConnectionMessage();
    ~ConnectionMessage();

    ConnectionMessage(const ConnectionMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  EclipseDebuggerInterface();
  ~EclipseDebuggerInterface();

 public:
  /* Methods */
  uint16_t port() const;
  void set_port(const uint16_t new_port);
  size_t maxlenof_port() const;
  char * host() const;
  void set_host(const char * new_host);
  size_t maxlenof_host() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
