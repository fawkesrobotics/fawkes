
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
    char skill_string[1024]; /**< 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
     */
  } SkillerInterface_data_t;

  SkillerInterface_data_t *data;

 public:
  /* messages */
  class CallSkillMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      char skill_string[1024]; /**< 
      Currently executed skill string, at least the first 1023 bytes of it.
      Must be properly null-terminated.
     */
    } CallSkillMessage_data_t;

    CallSkillMessage_data_t *data;

   public:
    CallSkillMessage(char * ini_skill_string);
    CallSkillMessage();
    ~CallSkillMessage();

    /* Methods */
    char * skill_string();
    void set_skill_string(const char * new_skill_string);
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

  virtual bool message_valid(const Message *message) const;
 private:
  SkillerInterface();
  ~SkillerInterface();

 public:
  virtual Message * create_message(const char *type) const;

  /* Methods */
  char * skill_string();
  void set_skill_string(const char * new_skill_string);

};

#endif
