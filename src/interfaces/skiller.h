
/***************************************************************************
 *  skiller.h - Fawkes BlackBoard Interface - SkillerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
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
    bool final; /**< 
      True if the execution of the current skill_string is final. False otherwise.
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

    /* Methods */
    char * skill_string();
    void set_skill_string(const char * new_skill_string);
    size_t maxlenof_skill_string() const;
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

    /* Methods */
    char * skill_string();
    void set_skill_string(const char * new_skill_string);
    size_t maxlenof_skill_string() const;
  };

  class RestartInterpreterMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
    } RestartInterpreterMessage_data_t;

    RestartInterpreterMessage_data_t *data;

   public:
    RestartInterpreterMessage();
    ~RestartInterpreterMessage();

    /* Methods */
  };

  class StopExecMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
    } StopExecMessage_data_t;

    StopExecMessage_data_t *data;

   public:
    StopExecMessage();
    ~StopExecMessage();

    /* Methods */
  };

  class AcquireControlMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
    } AcquireControlMessage_data_t;

    AcquireControlMessage_data_t *data;

   public:
    AcquireControlMessage();
    ~AcquireControlMessage();

    /* Methods */
  };

  class ReleaseControlMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
    } ReleaseControlMessage_data_t;

    ReleaseControlMessage_data_t *data;

   public:
    ReleaseControlMessage();
    ~ReleaseControlMessage();

    /* Methods */
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
  bool is_final();
  void set_final(const bool new_final);
  size_t maxlenof_final() const;

};

#endif
