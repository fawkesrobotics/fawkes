
/***************************************************************************
 *  SpeechSynthInterface.h - Fawkes BlackBoard Interface - SpeechSynthInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
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

#ifndef __INTERFACES_SPEECHSYNTHINTERFACE_H_
#define __INTERFACES_SPEECHSYNTHINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class SpeechSynthInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(SpeechSynthInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char text[1024]; /**< 
      Last spoken string. Must be properly null-terminated.
     */
    uint32_t msgid; /**< 
      The ID of the message that is currently being processed,
      or 0 if no message is being processed.
     */
    bool final; /**< 
      True, if the last text has been spoken, false if it is still running.
     */
    float duration; /**< 
      Length in seconds that it takes to speek the current text, -1 if
      unknown. This is the total duration of the current string, *not* the
      duration of already spoken or yet to speak text!
     */
  } SpeechSynthInterface_data_t;

  SpeechSynthInterface_data_t *data;

 public:
  /* messages */
  class SayMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char text[1024]; /**< 
      Last spoken string. Must be properly null-terminated.
     */
    } SayMessage_data_t;

    SayMessage_data_t *data;

   public:
    SayMessage(const char * ini_text);
    SayMessage();
    ~SayMessage();

    SayMessage(const SayMessage *m);
    /* Methods */
    char * text() const;
    void set_text(const char * new_text);
    size_t maxlenof_text() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  SpeechSynthInterface();
  ~SpeechSynthInterface();

 public:
  /* Methods */
  char * text() const;
  void set_text(const char * new_text);
  size_t maxlenof_text() const;
  uint32_t msgid() const;
  void set_msgid(const uint32_t new_msgid);
  size_t maxlenof_msgid() const;
  bool is_final() const;
  void set_final(const bool new_final);
  size_t maxlenof_final() const;
  float duration() const;
  void set_duration(const float new_duration);
  size_t maxlenof_duration() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
