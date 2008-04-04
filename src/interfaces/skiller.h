
/***************************************************************************
 *  skiller.h - Fawkes BlackBoard Interface - SkillerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory.
 */

#ifndef __INTERFACES_SKILLER_H_
#define __INTERFACES_SKILLER_H_

#include <interface/interface.h>
#include <interface/message.h>

class SkillerInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(SkillerInterface)
 /// @endcond
 public:
  /* constants */

  /** 
	This determines the current status of skill execution.
       */
  typedef enum {
    S_INACTIVE /**< No skill is running. */,
    S_FINAL /**< The skill string has been successfully processed. */,
    S_RUNNING /**< The execution is still running. */,
    S_FAILED /**< The execution failed and cannot succeed anymore. */
  } SkillStatusEnum;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int exclusive_controller; /**< 
      Instance serial of the exclusive controller of the skiller. If this does not
      carry your instance serial your exec messages will be ignored. Aquire control with
      the AquireControlMessage. Make sure you release control before exiting.
     */
    char skill_string[1024]; /**< 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
     */
    SkillStatusEnum status; /**< 
      The status of the current skill execution.
     */
    bool continuous; /**< 
      True if continuous execution is in progress, false if no skill string is executed
      at all or it is executed one-shot with ExecSkillMessage.
     */
  } SkillerInterface_data_t;

  SkillerInterface_data_t *data;

 public:
  /* messages */
  class ExecSkillMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      char skill_string[1024]; /**< 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
     */
    } ExecSkillMessage_data_t;

    ExecSkillMessage_data_t *data;

   public:
    ExecSkillMessage(const char * ini_skill_string);
    ExecSkillMessage();
    ~ExecSkillMessage();

    ExecSkillMessage(const ExecSkillMessage *m);
    /* Methods */
    char * skill_string();
    void set_skill_string(const char * new_skill_string);
    size_t maxlenof_skill_string() const;
    virtual Message * clone() const;
  };

  class ExecSkillContinuousMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      char skill_string[1024]; /**< 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
     */
    } ExecSkillContinuousMessage_data_t;

    ExecSkillContinuousMessage_data_t *data;

   public:
    ExecSkillContinuousMessage(const char * ini_skill_string);
    ExecSkillContinuousMessage();
    ~ExecSkillContinuousMessage();

    ExecSkillContinuousMessage(const ExecSkillContinuousMessage *m);
    /* Methods */
    char * skill_string();
    void set_skill_string(const char * new_skill_string);
    size_t maxlenof_skill_string() const;
    virtual Message * clone() const;
  };

  class RestartInterpreterMessage : public Message
  {
   public:
    RestartInterpreterMessage();
    ~RestartInterpreterMessage();

    RestartInterpreterMessage(const RestartInterpreterMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class StopExecMessage : public Message
  {
   public:
    StopExecMessage();
    ~StopExecMessage();

    StopExecMessage(const StopExecMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class AcquireControlMessage : public Message
  {
   public:
    AcquireControlMessage();
    ~AcquireControlMessage();

    AcquireControlMessage(const AcquireControlMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class ReleaseControlMessage : public Message
  {
   public:
    ReleaseControlMessage();
    ~ReleaseControlMessage();

    ReleaseControlMessage(const ReleaseControlMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  SkillerInterface();
  ~SkillerInterface();

 public:
  virtual Message * create_message(const char *type) const;

  /* Methods */
  char * skill_string();
  void set_skill_string(const char * new_skill_string);
  size_t maxlenof_skill_string() const;
  unsigned int exclusive_controller();
  void set_exclusive_controller(const unsigned int new_exclusive_controller);
  size_t maxlenof_exclusive_controller() const;
  SkillStatusEnum status();
  void set_status(const SkillStatusEnum new_status);
  size_t maxlenof_status() const;
  bool is_continuous();
  void set_continuous(const bool new_continuous);
  size_t maxlenof_continuous() const;

};

#endif
