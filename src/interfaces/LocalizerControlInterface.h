
/***************************************************************************
 *  LocalizerControlInterface.h - Fawkes BlackBoard Interface - LocalizerControlInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2009  Daniel Beck
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

#ifndef __INTERFACES_LOCALIZERCONTROLINTERFACE_H_
#define __INTERFACES_LOCALIZERCONTROLINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class LocalizerControlInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(LocalizerControlInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char map_name[30]; /**< The name of the current
    map */
  } LocalizerControlInterface_data_t;
#pragma pack(pop)

  LocalizerControlInterface_data_t *data;

 public:
  /* messages */
  class ResetMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float x; /**< The new initial x-coordinate. */
      float y; /**< The new initial x-coordinate. */
      float ori; /**< The new initial orientation. */
      float variance; /**< The variance for the reset position. */
    } ResetMessage_data_t;
#pragma pack(pop)

    ResetMessage_data_t *data;

   public:
    ResetMessage(const float ini_x, const float ini_y, const float ini_ori, const float ini_variance);
    ResetMessage();
    ~ResetMessage();

    ResetMessage(const ResetMessage *m);
    /* Methods */
    float x() const;
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y() const;
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float ori() const;
    void set_ori(const float new_ori);
    size_t maxlenof_ori() const;
    float variance() const;
    void set_variance(const float new_variance);
    size_t maxlenof_variance() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  LocalizerControlInterface();
  ~LocalizerControlInterface();

 public:
  /* Methods */
  char * map_name() const;
  void set_map_name(const char * new_map_name);
  size_t maxlenof_map_name() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
