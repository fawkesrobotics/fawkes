
/***************************************************************************
 *  mod_skiller.cpp -  OpenPRS skiller module
 *
 *  Created: Fri Aug 22 14:32:01 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <plugins/openprs/mod_utils.h>

#include <blackboard/remote.h>
#include <interfaces/SkillerInterface.h>
#include <utils/misc/string_conversions.h>

using namespace fawkes;

extern "C" void finalize();

// Global State
BlackBoard *blackboard;
SkillerInterface *skiller_if;

std::string             g_skill_string;
unsigned int            g_skill_msgid = 0;
Thread_Intention_Block *g_skill_tib = NULL;

std::string
gen_skill_string(TermList terms)
{
  int terms_len = sl_slist_length(terms);
  Term *name = (Term *)get_list_pos(terms, 1);
  std::string skill_string = std::string(name->u.string) + "{";
  for (int i = 2; i < terms_len; i += 2) {
    Term *key_t  = (Term *)get_list_pos(terms, i);
    Term *val_t  = (Term *)get_list_pos(terms, i+1);

    if (key_t->type != TT_ATOM && key_t->type != STRING) {
      fprintf(stderr, "Error: skill argument key neither of type "
	      "SYMBOL/ATOM nor STRING (%i)\n", key_t->type);
      return "";
    }

    const char *arg_key;
    if (i > 2) skill_string += ", ";

    arg_key = (key_t->type == TT_ATOM) ? key_t->u.id : key_t->u.string;
    skill_string += std::string(arg_key) + "=";

    switch (val_t->type) {
    case INTEGER:   skill_string += StringConversions::to_string(val_t->u.intval);     break;
    case LONG_LONG: skill_string += StringConversions::to_string((long int)val_t->u.llintval);   break;
    case TT_FLOAT:  skill_string += StringConversions::to_string(*val_t->u.doubleptr); break;
    case STRING:    skill_string += std::string("\"") + val_t->u.string + "\"";        break;
    case TT_ATOM:   skill_string += val_t->u.id;                                       break;
    case VARIABLE:
      {
	Envar *env;
	sl_loop_through_slist(global_var_list, env,  Envar *) {
	  if (strcmp(env->name, val_t->u.var->name) == 0) {
	    skill_string += env->value->u.string;
	  } else {
	    skill_string += "nil";
	  }
	}
	break;
      }
    default:
      fprintf(stderr, "Warning: unknown variable type for skill %s argument %s, using nil\n",
	      name->u.string, arg_key);
      skill_string += "nil";
    }
  }
  skill_string += "}";

  return skill_string;
}


bool
assert_exclusive_controller(unsigned int num_tries, unsigned int delay_msec)
{
  skiller_if->read();
  if (skiller_if->exclusive_controller() == skiller_if->serial())  return true;

  for (unsigned int i = 0; i < num_tries; ++i) {
    if (!skiller_if->has_writer())  return false;

    skiller_if->read();
    if (skiller_if->exclusive_controller() != skiller_if->serial()) {
      SkillerInterface::AcquireControlMessage *msg =
	new SkillerInterface::AcquireControlMessage(/* steal control */ false);
      skiller_if->msgq_enqueue(msg);
      usleep(delay_msec * 1000);
    } else {
      break;
    }
  }
  skiller_if->read();
  return (skiller_if->exclusive_controller() == skiller_if->serial());
}

extern "C"
Term *
action_skill_call(TermList terms)
{
  int terms_len = sl_slist_length(terms);
  if (terms_len == 0) {
    fprintf(stderr, "Error: no arguments to skill call\n");
    ACTION_FAIL();
  }

  Term *name;
  ACTION_SET_AND_ASSERT_ARG_TYPE("skill-call", name, terms, 1, STRING);

  if (terms_len % 2 == 0) {
    fprintf(stderr, "Error: invalid number of arguments (%i) to skill call for %s\n",
	    terms_len, name->u.string);
    ACTION_FAIL();
  }

  // there seems to be a race condition in OpenPRS. Without this mini sleep
  // action_first_call() would almost always return true
  usleep(500);
  if (action_first_call()) {
    skiller_if->read();
    if (!skiller_if->has_writer()) {
      fprintf(stderr, "Cannot send skill, interface has no writer\n");
      ACTION_FAIL();
    }
    if (! assert_exclusive_controller(20, 100)) {
      fprintf(stderr, "Cannot send skill, not exclusive controller\n");
      ACTION_FAIL();
    }

    std::string skill_string = gen_skill_string(terms);
    if (skill_string.empty()) {
      fprintf(stderr, "Error: failed to generate skill string\n");
      ACTION_FAIL();
    }

    printf("Calling skill %s\n", skill_string.c_str());

    SkillerInterface::ExecSkillMessage *msg =
      new SkillerInterface::ExecSkillMessage(skill_string.c_str());
    msg->ref();

    skiller_if->msgq_enqueue(msg);

    g_skill_msgid  = msg->id();
    g_skill_string = skill_string;

    msg->unref();

    g_skill_tib = current_tib;

    ACTION_WAIT();
  } else {
    if (current_tib != g_skill_tib) {
      fprintf(stderr, "Skill preempted by another skill, returning fail");
      ACTION_FAIL();
    }

    // we are called again due to :wait
    skiller_if->read();
    if (skiller_if->msgid() > g_skill_msgid) {
      fprintf(stderr, "Fail: a more recent message is being processed by the skiller (%u > %u)\n",
	      skiller_if->msgid(), g_skill_msgid);
      ACTION_FAIL();
    } else if (skiller_if->msgid() < g_skill_msgid) {
      // waiting to become active
      ACTION_WAIT();
    } else {
      // currently running
      switch (skiller_if->status()) {
      case SkillerInterface::S_FINAL:
	printf("Skill %s is FINAL\n", name->u.string);
	ACTION_FINAL();

      case SkillerInterface::S_FAILED:
      case SkillerInterface::S_INACTIVE:
	printf("Skill %s has FAILED\n", name->u.string);
	ACTION_FAIL();

      default: ACTION_WAIT();
      }
    }
  }
}



/** Entry function for the OpenPRS module. */
extern "C"
void init()
{
  printf("*** LOADING mod_skiller\n");

  std::string    fawkes_host;
  unsigned short fawkes_port = 0;
  get_fawkes_host_port(fawkes_host, fawkes_port);

  printf("Connecting to Fawkes at %s:%u\n", fawkes_host.c_str(), fawkes_port);
  try {
    blackboard = new RemoteBlackBoard(fawkes_host.c_str(), fawkes_port);
  } catch (Exception &e) {
    fprintf(stderr, "Error: cannot establish blackboard connection: %s\n",
            e.what_no_backtrace());
  }

  skiller_if = blackboard->open_for_reading<SkillerInterface>("Skiller");

  printf("Acquiring exclusive skiller control\n");
  SkillerInterface::AcquireControlMessage *msg =
    new SkillerInterface::AcquireControlMessage(/* steal control */ true);
  skiller_if->msgq_enqueue(msg);

  declare_atom("true");
  declare_atom("false");
  make_and_declare_action("skill-call", action_skill_call, -1);
  add_user_end_kernel_hook(finalize);
}

/** Finalization function for the OpenPRS module. */
extern "C"
void finalize()
{
  printf("*** DESTROYING mod_skiller\n");
  if (skiller_if->has_writer()) {
    SkillerInterface::ReleaseControlMessage *msg =
      new SkillerInterface::ReleaseControlMessage();
    skiller_if->msgq_enqueue(msg);
    usleep(100000);
  }

  blackboard->close(skiller_if);
  usleep(100000);
  delete blackboard;
}
