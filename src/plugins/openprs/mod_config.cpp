
/***************************************************************************
 *  mod_config.cpp -  OpenPRS config module
 *
 *  Created: Fri Sep 05 13:00:11 2014
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

// this must come first due to a define of enqueue in OpenPRS' slistPack_f.h
#include <netcomm/fawkes/client.h>

#include <plugins/openprs/mod_utils.h>
#include <config/netconf.h>
#include <memory>
#include <oprs_f-pub.h>

using namespace fawkes;

extern "C" void finalize();

// Global variables
FawkesNetworkClient  *g_fnet_client = NULL;
NetworkConfiguration *g_config = NULL;


extern "C"
Term *
action_config_load(TermList terms)
{
  Term *prefix;
  ACTION_ASSERT_ARG_LENGTH("config-load", terms, 1);
  ACTION_SET_AND_ASSERT_ARG_TYPE("config-load", prefix, terms, 1, STRING);

#if __cplusplus >= 201103L
  std::unique_ptr<Configuration::ValueIterator> v(g_config->search(prefix->u.string));
#else
  std::auto_ptr<Configuration::ValueIterator> v(g_config->search(prefix->u.string));
#endif
  while (v->next()) {
    TermList tl = sl_make_slist();
    tl = build_term_list(tl, build_string(v->path()));
    

    std::string type = "";
    std::string value = v->get_as_string();

    if      (v->is_uint()) {
      tl = build_term_list(tl, build_id(declare_atom("UINT")));
      if (v->is_list()) {
	TermList ll = sl_make_slist();
	std::vector<unsigned int> uints = v->get_uints();
	for (size_t i = 0; i < uints.size(); ++i) {
	  ll = build_term_list(ll, build_long_long(uints[i]));
	}
	tl = build_term_list(tl, build_term_l_list_from_c_list(ll));
      } else {
	tl = build_term_list(tl, build_long_long(v->get_uint()));
      }
    } else if (v->is_int()) {
      tl = build_term_list(tl, build_id(declare_atom("INT")));
      if (v->is_list()) {
	TermList ll = sl_make_slist();
	std::vector<int> ints = v->get_ints();
	for (size_t i = 0; i < ints.size(); ++i) {
	  ll = build_term_list(ll, build_integer(ints[i]));
	}
	tl = build_term_list(tl, build_term_l_list_from_c_list(ll));
      } else {
	tl = build_term_list(tl, build_integer(v->get_int()));
      }
    } else if (v->is_float()) {
      tl = build_term_list(tl, build_id(declare_atom("FLOAT")));
      if (v->is_list()) {
	TermList ll = sl_make_slist();
	std::vector<float> floats = v->get_floats();
	for (size_t i = 0; i < floats.size(); ++i) {
	  ll = build_term_list(ll, build_float(floats[i]));
	}
	tl = build_term_list(tl, build_term_l_list_from_c_list(ll));
      } else {
	tl = build_term_list(tl, build_float(v->get_float()));
      }
    } else if (v->is_bool()) {
      tl = build_term_list(tl, build_id(declare_atom("BOOL")));
      if (v->is_list()) {
	TermList ll = sl_make_slist();
	std::vector<bool> bools = v->get_bools();
	for (size_t i = 0; i < bools.size(); ++i) {
	  ll = build_term_list(ll, bools[i] ? build_t() : build_nil());
	}
	tl = build_term_list(tl, build_term_l_list_from_c_list(ll));
      } else {
	tl = build_term_list(tl, v->get_bool() ? build_t() : build_nil());
      }
    } else if (v->is_string()) {
      tl = build_term_list(tl, build_id(declare_atom("STRING")));
      if (v->is_list()) {
	TermList ll = sl_make_slist();
	std::vector<std::string> strings = v->get_strings();
	for (size_t i = 0; i < strings.size(); ++i) {
	  ll = build_term_list(ll, build_string(strings[i].c_str()));
	}
	tl = build_term_list(tl, build_term_l_list_from_c_list(ll));
      } else {
	tl = build_term_list(tl, build_string(v->get_string().c_str()));
      }
    } else {
      fprintf(stderr, "Warn[config-load]: value at '%s' of unknown type '%s'",
	      v->path(), v->type());
    }

    add_external_fact((char *)"confval", tl);
  }

  TermList tl = sl_make_slist();
  tl = build_term_list(tl, build_string(prefix->u.string));
  add_external_fact((char *)"config-loaded", tl);

  ACTION_FINAL();
}


extern "C"
PBoolean
pred_string_prefix_p(TermList terms)
{
  Term *str, *prefix;
  ACTION_ASSERT_B_ARG_LENGTH("string-prefix-p", terms, 2);
  ACTION_SET_AND_ASSERT_B_ARG_TYPE("string-prefix-p", str, terms, 1, STRING);
  ACTION_SET_AND_ASSERT_B_ARG_TYPE("string-prefix-p", prefix, terms, 2, STRING);

  return (strncmp(str->u.string, prefix->u.string, strlen(prefix->u.string)) == 0);
}

extern "C"
Term *
func_string_remove_prefix(TermList terms)
{
  Term *str, *prefix;
  ACTION_ASSERT_ARG_LENGTH("string-remove-prefix", terms, 2);
  ACTION_SET_AND_ASSERT_ARG_TYPE("string-remove-prefix", str, terms, 1, STRING);
  ACTION_SET_AND_ASSERT_ARG_TYPE("string-remove-prefix", prefix, terms, 2, STRING);

  if (! pred_string_prefix_p(terms))
    return build_string(str->u.string);

  if (strlen(prefix->u.string) >= strlen(str->u.string))
    return build_string("");

  return build_string(&str->u.string[strlen(prefix->u.string)]);
}


/** Entry function for the OpenPRS module. */
extern "C"
void init()
{
  printf("*** LOADING mod_config\n");

  std::string    fawkes_host;
  unsigned short fawkes_port = 0;
  get_fawkes_host_port(fawkes_host, fawkes_port);

  printf("Connecting to Fawkes at %s:%u\n", fawkes_host.c_str(), fawkes_port);
  try {
    g_fnet_client = new FawkesNetworkClient(fawkes_host.c_str(), fawkes_port);
    g_fnet_client->connect();
    g_config      = new NetworkConfiguration(g_fnet_client);
    g_config->set_mirror_mode(true);
  } catch (Exception &e) {
    fprintf(stderr, "Error: cannot establish network connection: %s\n",
            e.what_no_backtrace());
  }

  make_and_declare_action("config-load", action_config_load, 1);
  make_and_declare_eval_pred("string-prefix-p", pred_string_prefix_p, 2, FALSE);
  make_and_declare_eval_funct("string-remove-prefix", func_string_remove_prefix, 2);
  add_user_end_kernel_hook(finalize);
}

/** Finalization function for the OpenPRS module. */
extern "C"
void finalize()
{
  printf("*** DESTROYING mod_config\n");
  delete g_config;
  g_config = NULL;
  delete g_fnet_client;
  g_fnet_client = NULL;
}
