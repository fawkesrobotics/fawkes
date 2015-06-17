  
/***************************************************************************
 *  mod_blackboard.cpp -  OpenPRS blackboard module
 *
 *  Created: Tue Sep 02 10:38:03 2014
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
#include <utils/misc/string_conversions.h>
#include <utils/time/time.h>

#include <oprs-type_f-pub.h>
#include <oprs-array_f-pub.h>
#include <oprs_f-pub.h>
#include <slistPack_f.h>
#include <lisp-list_f-pub.h>

using namespace fawkes;

extern "C" void finalize();


// Global variables
BlackBoard                         *g_blackboard = NULL;
std::map<std::string, Interface *>  g_interfaces_read;
std::map<std::string, Interface *>  g_interfaces_write;
Symbol                              g_bb_read_sym;
Symbol                              g_bb_write_sym;
Symbol                              g_bb_data_sym;

extern "C"
Term *
action_blackboard_open(TermList terms)
{
  int terms_len = sl_slist_length(terms);
  if (terms_len != 3) {
    fprintf(stderr, "Error[bb-open-interface]: invalid number of "
	    "arguments: req 3, got %i\n", terms_len);
    ACTION_FAIL();
  }

  Term *type = (Term *)get_list_pos(terms, 1);
  Term *id   = (Term *)get_list_pos(terms, 2);
  Term *mode = (Term *)get_list_pos(terms, 3);
  if (type->type != STRING) {
    fprintf(stderr, "Error[bb-open-interface]: interface type is not a STRING\n");
    ACTION_FAIL();
  }
  if (id->type != STRING) {
    fprintf(stderr, "Error[bb-open-interface]: interface ID is not a STRING\n");
    ACTION_FAIL();
  }
  if (id->type != STRING) {
    fprintf(stderr, "Error[bb-open-interface]: interface ID is not a STRING\n");
    ACTION_FAIL();
  }
  if (mode->type != TT_ATOM) {
    fprintf(stderr, "Error[bb-open-interface]: interface mode is not a symbol\n");
    ACTION_FAIL();
  }
  if (mode->u.id != g_bb_read_sym && mode->u.id != g_bb_write_sym) {
    fprintf(stderr, "Error[bb-open-interface]: interface mode must be BB-READ or BB-WRITE\n");
    ACTION_FAIL();
  }

  std::string uid = std::string(type->u.string) + "::" + id->u.string;

  if (mode->u.id == g_bb_read_sym) {
    if (g_interfaces_read.find(uid) == g_interfaces_read.end()) {
      try {
	printf("Opening interface %s::%s for reading\n", type->u.string, id->u.string);
	Interface *iface = g_blackboard->open_for_reading(type->u.string, id->u.string);
	g_interfaces_read[uid] = iface;
      } catch (Exception &e) {
	fprintf(stderr, "Failed to open interface %s::%s: %s",
		type->u.string, id->u.string, e.what_no_backtrace());
	ACTION_FAIL();
      }
    }
  } else {
    if (g_interfaces_write.find(uid) == g_interfaces_write.end()) {
      try {
	printf("Opening interface %s::%s for writing\n", type->u.string, id->u.string);
	Interface *iface = g_blackboard->open_for_writing(type->u.string, id->u.string);
	g_interfaces_write[uid] = iface;
      } catch (Exception &e) {
	fprintf(stderr, "Failed to open interface %s::%s: %s\n",
		type->u.string, id->u.string, e.what_no_backtrace());
	ACTION_FAIL();
      }
    }
  }

  ACTION_FINAL();
}


extern "C"
Term *
action_blackboard_close(TermList terms)
{
  int terms_len = sl_slist_length(terms);
  if (terms_len != 2) {
    fprintf(stderr, "Error[bb-close-interface]: invalid number of "
	    "arguments: req 2, got %i\n", terms_len);
    ACTION_FAIL();
  }

  Term *type = (Term *)get_list_pos(terms, 1);
  Term *id   = (Term *)get_list_pos(terms, 2);
  if (type->type != STRING) {
    fprintf(stderr, "Error[bb-close-interface]: interface type is not a STRING\n");
    ACTION_FAIL();
  }
  if (id->type != STRING) {
    fprintf(stderr, "Error[bb-close-interface]: interface ID is not a STRING\n");
    ACTION_FAIL();
  }
  if (id->type != STRING) {
    fprintf(stderr, "Error[bb-close-interface]: interface ID is not a STRING\n");
    ACTION_FAIL();
  }

  std::string uid = std::string(type->u.string) + "::" + id->u.string;

  if (g_interfaces_read.find(uid) != g_interfaces_read.end()) {
    try {
      printf("Closing reading interface %s::%s\n", type->u.string, id->u.string);
      g_blackboard->close(g_interfaces_read[uid]);
      g_interfaces_read.erase(uid);
    } catch (Exception &e) {
	fprintf(stderr, "Failed to close interface %s::%s: %s",
		type->u.string, id->u.string, e.what_no_backtrace());
	ACTION_FAIL();
    }
  } else if (g_interfaces_write.find(uid) != g_interfaces_write.end()) {
    try {
      printf("Closing writing interface %s::%s\n", type->u.string, id->u.string);
      g_blackboard->close(g_interfaces_write[uid]);
      g_interfaces_write.erase(uid);
    } catch (Exception &e) {
      fprintf(stderr, "Failed to close interface %s::%s: %s\n",
	      type->u.string, id->u.string, e.what_no_backtrace());
      ACTION_FAIL();
    }
  }

  ACTION_FINAL();
}


extern "C"
Term *
action_blackboard_print(TermList terms)
{
  Term *type = (Term *)get_list_pos(terms, 1);
  Term *id   = (Term *)get_list_pos(terms, 2);
  if (type->type != STRING) {
    fprintf(stderr, "Error[bb-print]: interface type is not a STRING\n");
    ACTION_FAIL();
  }
  if (id->type != STRING) {
    fprintf(stderr, "Error[bb-print]: interface ID is not a STRING\n");
    ACTION_FAIL();
  }
  if (id->type != STRING) {
    fprintf(stderr, "Error[bb-print]: interface ID is not a STRING\n");
    ACTION_FAIL();
  }

  std::string uid = std::string(type->u.string) + "::" + id->u.string;
  //printf("*** Called to print %s\n", uid.c_str());

  Interface *i = NULL;
  if (g_interfaces_read.find(uid) != g_interfaces_read.end()) {
    i = g_interfaces_read[uid];
  } else if (g_interfaces_write.find(uid) != g_interfaces_write.end()) {
    i = g_interfaces_write[uid];
  } else {
    fprintf(stderr, "Error[bb-print]: interface %s has not been opened\n", uid.c_str());
    fprintf(stderr, "Error[bb-print]: Open interfaces are:\n");
    for (auto j : g_interfaces_read) {
      fprintf(stderr, "Error[bb-print]: [R] %s\n", j.second->uid());
    }
    for (auto j : g_interfaces_write) {
      fprintf(stderr, "Error[bb-print]: [W] %s\n", j.second->uid());
    }
    fprintf(stderr, "Error[bb-print]: -----\n");
    ACTION_FAIL();
  }

  try {
    i->read();
    const Time *t = i->timestamp();

    std::string fact = std::string("(bb-data \"type\" \"") + i->type() + "\"" +
      " \"id\" \"" + i->id() + "\"" +
      " \"time\" " + StringConversions::to_string(t->get_sec()) + " "
      + StringConversions::to_string(t->get_usec()) + ""
      + " (. ";

    InterfaceFieldIterator f, f_end = i->fields_end();
    for (f = i->fields(); f != f_end; ++f) {
      std::string value;
      if (f.get_type() == IFT_STRING) {
	value = f.get_value_string();
	std::string::size_type pos = 0;
	while ((pos = value.find("\"", pos)) != std::string::npos) {
	  value.replace(pos, 1, "\\\"");
	  pos += 2;
	}
	  value = std::string("\"") + value + "\"";
      } else if (f.get_type() == IFT_ENUM) {
	  value = std::string("\"") + f.get_value_string(" ") + "\"";
      } else {
	value = f.get_value_string(" ");
	std::string::size_type pos;
	while ((pos = value.find(",")) != std::string::npos) {
	  value = value.erase(pos, 1);
	}
      }
      if (f.get_length() > 1) {
	fact += std::string(" \"") + f.get_name() + "\" [ " + value + " ]";
      } else {
	fact += std::string(" \"") + f.get_name() + "\" " + value;
      }
    }
    fact += " .))";
    //envs_[env_name]->assert_fact(fact);
    printf("%s\n", fact.c_str());
  } catch (Exception &e) {
    fprintf(stderr, "Error[bb-print]: %s\n", e.what_no_backtrace());
    ACTION_FAIL();
  }
  ACTION_FINAL();
}


#define ADD_ARRAY(src_type, target_type, array_type)			\
  do {									\
    target_type *array = (target_type *)OPRS_MALLOC(sizeof(target_type) * f.get_length());					\
    src_type ## _t *src_array = f.get_ ## src_type ## s();		\
    for (unsigned int j = 0; j < f.get_length(); ++j)  array[j] = src_array[j]; \
    data = l_add_to_tail(data, make_ ## array_type ## _array_from_array(f.get_length(), array)); \
  } while (0);

#define BUILD_FUNC(singular_type) build_ ## singular_type
#define GET_FUNC(src_type) get_ ## src_type

#define ADD_NUM_DATA(src_type, target_type, array_type, singular_type)	\
  do {									\
    if (f.get_length() > 1) {						\
      ADD_ARRAY(src_type, target_type, array_type);			\
    } else {								\
      data = l_add_to_tail(data, BUILD_FUNC(singular_type)(f.GET_FUNC(src_type)())); \
    }									\
  } while (0);
  

static void
post_interface(Interface *i)
{
  i->read();
  if (i->changed()) {
    const Time *t = i->timestamp();

    TermList tl = sl_make_slist();
    tl = build_term_list(tl, build_string("type"));
    tl = build_term_list(tl, build_string(i->type()));
    tl = build_term_list(tl, build_string("id"));
    tl = build_term_list(tl, build_string(i->id()));
    tl = build_term_list(tl, build_string("time"));
    tl = build_term_list(tl, build_long_long(t->get_sec()));
    tl = build_term_list(tl, build_long_long(t->get_usec()));

    L_List data = l_nil;
    InterfaceFieldIterator f, f_end = i->fields_end();
    for (f = i->fields(); f != f_end; ++f) {
      data = l_add_to_tail(data, build_string(f.get_name()));

      switch (f.get_type()) {
      case IFT_BOOL:
	data = l_add_to_tail(data, build_id(f.get_bool() ? lisp_t_sym : nil_sym));
	break;
      case IFT_INT8:
	ADD_NUM_DATA(int8, int, int, integer);
	break;
      case IFT_UINT8:
	ADD_NUM_DATA(uint8, int, int, integer);
	break;
      case IFT_INT16:
	ADD_NUM_DATA(int16, int, int, integer);
	break;
      case IFT_UINT16:
	ADD_NUM_DATA(uint16, int, int, integer);
	break;
      case IFT_INT32:
	ADD_NUM_DATA(int32, int, int, integer);
	break;
      case IFT_UINT32:
	ADD_NUM_DATA(uint32, double, float, long_long);
	break;
      case IFT_INT64:
	ADD_NUM_DATA(int64, double, float, long_long);
	break;
      case IFT_UINT64:
	ADD_NUM_DATA(uint64, double, float, long_long);
	break;
      case IFT_FLOAT:
	ADD_NUM_DATA(float, double, float, float);
	break;
      case IFT_DOUBLE:
	ADD_NUM_DATA(double, double, float, float);
	break;
      case IFT_STRING:
	data = l_add_to_tail(data, build_string(f.get_value_string()));
	break;
      case IFT_BYTE:
	ADD_NUM_DATA(uint8, int, int, integer);
	break;
      case IFT_ENUM:
	data = l_add_to_tail(data, build_string(f.get_value_string()));
	break;
      }

    }

    tl = build_term_list(tl, build_l_list(data));
    add_external_fact((char *)"bb-data", tl);
  }
}


extern "C"
Term *
action_blackboard_read_all(TermList terms)
{
  try {
    for (auto &if_entry : g_interfaces_read) {
      Interface *i = if_entry.second;
      post_interface(i);
    }
  } catch (Exception &e) {
    fprintf(stderr, "Error[bb-read]: read failed: %s\n", e.what_no_backtrace());
    ACTION_FAIL();
  }
  ACTION_FINAL();
}


extern "C"
Term *
action_blackboard_read(TermList terms)
{
  int terms_len = sl_slist_length(terms);
  if (terms_len != 2) {
    fprintf(stderr, "Error[bb-read]: invalid number of "
	    "arguments: req 2, got %i\n", terms_len);
    ACTION_FAIL();
  }

  Term *type = (Term *)get_list_pos(terms, 1);
  Term *id   = (Term *)get_list_pos(terms, 2);
  if (type->type != STRING) {
    fprintf(stderr, "Error[bb-read]: interface type is not a STRING\n");
    ACTION_FAIL();
  }
  if (id->type != STRING) {
    fprintf(stderr, "Error[bb-read]: interface ID is not a STRING\n");
    ACTION_FAIL();
  }
  if (id->type != STRING) {
    fprintf(stderr, "Error[bb-read]: interface ID is not a STRING\n");
    ACTION_FAIL();
  }

  std::string uid = std::string(type->u.string) + "::" + id->u.string;

  if (g_interfaces_read.find(uid) != g_interfaces_read.end()) {
    try {
      post_interface(g_interfaces_read[uid]);
    } catch (Exception &e) {
      fprintf(stderr, "Failed to read interface %s::%s: %s",
	      type->u.string, id->u.string, e.what_no_backtrace());
      ACTION_FAIL();
    }
  } else {
    fprintf(stderr, "Failed to read interface %s::%s: interface not opened",
	    type->u.string, id->u.string);
    ACTION_FAIL();
  }

  ACTION_FINAL();
}


/** Searches for a given entry in the bb-date object
 *  specified by String. Returns nil if entry not present */
extern "C"
Term *
func_blackboard_value(TermList terms)
{
  int terms_len = sl_slist_length(terms);
  if (terms_len != 2) {
    fprintf(stderr, "Error[bb-value]: invalid number of "
	    "arguments: req 2, got %i\n", terms_len);
    ACTION_FAIL(); 
  }
  Term *dlist = (Term *)get_list_pos(terms, 1);
  Term *name  = (Term *)get_list_pos(terms, 2);
  Term *restemp;
  
  if (dlist->type != LISP_LIST) {
    fprintf(stderr, "Error[bb-value]: first argument is not a LISP_LIST\n");
    ACTION_FAIL();
  }
  if (name->type != STRING) {
    fprintf(stderr, "Error[bb-value]: interface ID is not a STRING\n");
    ACTION_FAIL();
  }
  char* pattern = name->u.string;
  int i = 1;
  while (i < l_length(dlist->u.l_list) - 1) {
    Term *t1 = get_term_from_l_car(l_nth(dlist->u.l_list, i));
    t1 = t1;
    if (t1->type == STRING) {             
      char* searched = t1->u.string;
      if (strcmp(pattern, searched) == 0) {
	restemp = get_term_from_l_car(l_nth((dlist->u).l_list, i+1)); 
	// cast string objects to symbols to prevent upper-/lowercase
	// differences in db.
	if (restemp->type == STRING) {
	  std::string outputs = std::string(restemp->u.string);
	  std::transform(outputs.begin(), outputs.end(), outputs.begin(), ::tolower);
	  restemp = build_id(declare_atom(outputs.c_str())); 
	}
	return copy_term(restemp);
      }
      ++i;
    }
  }
  fprintf(stderr, "Error[bb-value]: wanted entry in bb-data not present\n");
  ACTION_FAIL();
}

/** Entry function for the OpenPRS module. */
extern "C"
void init()
{
  printf("*** LOADING mod_blackboard\n");

  std::string    fawkes_host;
  unsigned short fawkes_port = 0;
  get_fawkes_host_port(fawkes_host, fawkes_port);

  printf("Connecting to Fawkes at %s:%u\n", fawkes_host.c_str(), fawkes_port);
  try {
    g_blackboard = new RemoteBlackBoard(fawkes_host.c_str(), fawkes_port);
  } catch (Exception &e) {
    fprintf(stderr, "Error: cannot establish blackboard connection: %s\n",
            e.what_no_backtrace());
  }

  g_bb_read_sym  = declare_atom("BB-READ");
  g_bb_write_sym = declare_atom("BB-WRITE");
  g_bb_data_sym  = declare_atom("bb-data");
  declare_pred_from_symbol(g_bb_data_sym);
  make_and_declare_eval_funct("bb-value", func_blackboard_value ,2);
  make_and_declare_action("bb-open", action_blackboard_open, 3);
  make_and_declare_action("bb-close", action_blackboard_close, 2);
  make_and_declare_action("bb-read", action_blackboard_read, 2);
  make_and_declare_action("bb-read-all", action_blackboard_read_all, 0);
  make_and_declare_action("bb-print", action_blackboard_print, 2);
  add_user_end_kernel_hook(finalize);
}

/** Finalization function for the OpenPRS module. */
extern "C"
void finalize()
{
  printf("*** DESTROYING mod_skiller\n");
  for (auto &iface : g_interfaces_read) {
    g_blackboard->close(iface.second);
  }
  g_interfaces_read.clear();
  for (auto &iface : g_interfaces_write) {
    g_blackboard->close(iface.second);
  }
  g_interfaces_write.clear();

  delete g_blackboard;
  g_blackboard = NULL;
}
