
/***************************************************************************
 *  mod_utils.cpp -  OpenPRS module utils
 *
 *  Created: Tue Aug 26 17:33:01 2014
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

#ifndef __PLUGINS_OPENPRS_AGENT_MOD_UTILS_H_
#define __PLUGINS_OPENPRS_AGENT_MOD_UTILS_H_

#include <cstdio>

#include <constant-pub.h>
#include <opaque-pub.h>
#include <oprs-type-pub.h>
#include <oprs-type_f-pub.h>
#include <macro-pub.h>
#include <slistPack-pub.h>
#include <shashPack_f.h>
#include <user-end-hook_f-pub.h>
#include <action_f-pub.h>
#include <intention_f-pub.h>
#include <ev-function_f-pub.h>
#include <ev-predicate_f-pub.h>

#include <string>
#include <algorithm>
#include <cstring>
#include <unistd.h>

/// @cond EXTERNAL
extern "C" {
  typedef Slist *List_Envar;
  List_Envar global_var_list;
  Shash *id_hash;

  typedef struct type Type;
  typedef Slist *TypeList;
  typedef Slist *SymList;
  Symbol wait_sym;

  /* type de Variable */
  typedef enum {LOGICAL_VARIABLE, PROGRAM_VARIABLE} Variable_Type;

  struct envar {			/* Un envar */
    Symbol name;		/* son name */
    Term *value;		/* le term sur lequel elle pointe */
    Type *unif_type;		
    Variable_Type type BITFIELDS(:8);	/* Le type de la variable */
  };

  struct type {
    Symbol name;
    Type *father;
    TypeList sur_types;	/* Plus itself */
    SymList elts;
  };
}
/// @endcond

/** Get Fawkes host and port.
 * This goes through the list of global variables and extracts the
 * parent Fawkes hostname from @@FAWKES_HOST and the TCP port from
 * @@FAWKES_PORT.
 * @return true if the data could be gathered successfully, false otherwise
 */
bool
get_fawkes_host_port(std::string &fawkes_host, unsigned short &fawkes_port)
{
  Envar *env;
  sl_loop_through_slist(global_var_list, env,  Envar *) {
    if (strcmp(env->name, "@@FAWKES_HOST") == 0 || strcmp(env->name, "@@fawkes_host") == 0) {
      if (env->value->type != STRING) {
	fprintf(stderr, "Error: @@FAWKES_HOST is not of type STRING\n");
	return false;
      }
      fawkes_host = env->value->u.string;
    } else if (strcmp(env->name, "@@FAWKES_PORT") == 0 || strcmp(env->name, "@@fawkes_port") == 0) {
      if (env->value->type != STRING) {
	fprintf(stderr, "Error: @@FAWKES_PORT is not of type STRING\n");
	return false;
      }
      fawkes_port = atoi(env->value->u.string);
    }
  }

  if (fawkes_host.empty()) {
    fawkes_host = getenv("FAWKES_HOST");
  }
  if (fawkes_port == 0) {
    fawkes_port = atoi(getenv("FAWKES_PORT"));
  }

  return (! fawkes_host.empty() && fawkes_port != 0);
}

//define ACTION_DEBUG

#ifdef ACTION_DEBUG
#  define ACTION_RETURN(value)						\
  do {									\
    if (value == nil_sym || value == lisp_t_sym) {			\
      printf("Action returns: %s\n", value == nil_sym ? "FAIL" : (value == wait_sym ? ":WAIT" : "FINAL")); \
    }									\
    Term *res = MAKE_OBJECT(Term); res->type = ATOM; res->u.id = value; return res; \
  } while (0);
#else
#  define ACTION_RETURN(value)						\
  do { Term *res = MAKE_OBJECT(Term); res->type = ATOM; res->u.id = value; return res; } while (0);
#endif
#define ACTION_FAIL()  ACTION_RETURN(nil_sym);
#define ACTION_WAIT()  ACTION_RETURN(wait_sym);
#define ACTION_FINAL() ACTION_RETURN(lisp_t_sym);

bool
assert_arg_type(const char *func_name, TermList &tl, int index, Term_Type t_type)
{
  Term *t = (Term *)get_list_pos(tl, index);
  if (t->type != t_type) {
    const char *type = "UNKNOWN";
    switch (t_type) {
    case INTEGER:        type = "INTEGER";        break;
    case LONG_LONG:      type = "LONG_LONG";      break;
    case TT_FLOAT:       type = "TT_FLOAT";       break;
    case STRING:         type = "STRING";         break;
    case TT_ATOM:        type = "TT_ATOM";        break;
    case EXPRESSION:     type = "EXPRESSION";     break;
    case VARIABLE:       type = "VARIABLE";       break;
    case LISP_LIST:      type = "LISP_LIST";      break;
    case INT_ARRAY:      type = "INT_ARRAY";      break;
    case FLOAT_ARRAY:    type = "FLOAT_ARRAY";    break;
    case C_LIST:         type = "C_LIST";         break;
    case TT_FACT:        type = "TT_FACT";        break;
    case TT_GOAL:        type = "FF_GOAL";        break; 
    case TT_INTENTION:   type = "TT_INTENTION";   break;
    case TT_OP_INSTANCE: type = "TT_OP_INSTANCE"; break;
    case U_POINTER:      type = "U_POINTER";      break;
    case U_MEMORY:       type = "U_MEMORY";       break;
    }
    fprintf(stderr, "Error[%s]: argument type is not a %s\n",
	    func_name, type);
    return false;
  } else {
    return true;
  }
}

#define ACTION_ASSERT_ARG_LENGTH(func_name, tl, length)			\
  do {									\
    int terms_len = sl_slist_length(tl);				\
    if (terms_len != length) {						\
      fprintf(stderr, "Error[%s]: invalid number of arguments:"		\
	      " req %i, got %i\n", func_name, length, terms_len);	\
      ACTION_FAIL();							\
    }									\
  } while (0);

#define ACTION_SET_AND_ASSERT_ARG_TYPE(func_name, var, tl, index, t_type) \
  do {									\
    if (! assert_arg_type(func_name, tl, index, t_type)) ACTION_FAIL();	\
    var = (Term *)get_list_pos(tl, index);				\
  } while (0);


// Boolean return versions
#define ACTION_ASSERT_B_ARG_LENGTH(func_name, tl, length)		\
  do {									\
    int terms_len = sl_slist_length(tl);				\
    if (terms_len != length) {						\
      fprintf(stderr, "Error[%s]: invalid number of arguments:"		\
	      " req %i, got %i\n", func_name, length, terms_len);	\
      return false;							\
    }									\
  } while (0);

#define ACTION_SET_AND_ASSERT_B_ARG_TYPE(func_name, var, tl, index, t_type) \
  do {									\
    if (! assert_arg_type(func_name, tl, index, t_type)) return false;	\
    var = (Term *)get_list_pos(tl, index);				\
  } while (0);

#define ACTION_ASSERT_B_ARG_TYPE(func_name, var, tl, index, t_type) \
  do { if (! assert_arg_type(func_name, tl, index, t_type)) return false; } while (0);

#endif
