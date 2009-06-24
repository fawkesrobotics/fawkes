
/***************************************************************************
 *  SpeechRecognitionInterface.h - Fawkes BlackBoard Interface - SpeechRecognitionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2009  Tim Niemueller and Masrur Doostdar
 *
 *  $Id$
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

#ifndef __INTERFACES_SPEECHRECOGNITIONINTERFACE_H_
#define __INTERFACES_SPEECHRECOGNITIONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>

namespace fawkes {

class SpeechRecognitionInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(SpeechRecognitionInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int counter; /**< 
      Counter for messages. Increased after each new recognized string.
     */
    bool processing; /**< 
      True, if the the speech recognition is currently processing.
     */
    bool enabled; /**< 
      True, if speech processing is currently enabled, false otherwise.
     */
    char text[1024]; /**< 
      Last spoken string. Must be properly null-terminated.
     */
  } SpeechRecognitionInterface_data_t;

  SpeechRecognitionInterface_data_t *data;

 public:
  /* messages */
  class ResetMessage : public Message
  {
   public:
    ResetMessage();
    ~ResetMessage();

    ResetMessage(const ResetMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class SetEnabledMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      bool enabled; /**< 
      True, if speech processing is currently enabled, false otherwise.
     */
    } SetEnabledMessage_data_t;

    SetEnabledMessage_data_t *data;

   public:
    SetEnabledMessage(const bool ini_enabled);
    SetEnabledMessage();
    ~SetEnabledMessage();

    SetEnabledMessage(const SetEnabledMessage *m);
    /* Methods */
    bool is_enabled() const;
    void set_enabled(const bool new_enabled);
    size_t maxlenof_enabled() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  SpeechRecognitionInterface();
  ~SpeechRecognitionInterface();

 public:
  /* Methods */
  char * text() const;
  void set_text(const char * new_text);
  size_t maxlenof_text() const;
  unsigned int counter() const;
  void set_counter(const unsigned int new_counter);
  size_t maxlenof_counter() const;
  bool is_processing() const;
  void set_processing(const bool new_processing);
  size_t maxlenof_processing() const;
  bool is_enabled() const;
  void set_enabled(const bool new_enabled);
  size_t maxlenof_enabled() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);

};

} // end namespace fawkes

#endif
