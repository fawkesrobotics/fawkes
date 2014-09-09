
/***************************************************************************
 *  mod_protobuf.cpp -  OpenPRS protobuf communication module
 *
 *  Created: Tue Sep 02 16:23:43 2014
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
#include "oprs_protobuf.h"

#include <core/exception.h>

#include <oprs-type_f-pub.h>
#include <oprs-array_f-pub.h>
#include <oprs_f-pub.h>
#include <slistPack_f.h>
#include <lisp-list_f-pub.h>

using namespace oprs_protobuf;

extern "C" void finalize();

#define ASSERT_PB						\
  do {								\
    if (! g_oprs_pb) {						\
      fprintf(stderr, "Error: pb-setup has not been called\n");	\
      ACTION_FAIL();						\
    }								\
  } while (0);

#define ASSERT_B_PB						\
  do {								\
    if (! g_oprs_pb) {						\
      fprintf(stderr, "Error: pb-setup has not been called\n");	\
      return true;						\
    }								\
  } while (0);


// Global variables
OpenPRSProtobuf                     *g_oprs_pb = NULL;

extern "C"
Term *
action_pb_setup(TermList terms)
{
  int terms_len = sl_slist_length(terms);
  if (terms_len != 1) {
    fprintf(stderr, "Error[pb-setup]: invalid number of "
	    "arguments: req 1, got %i\n", terms_len);
    ACTION_FAIL();
  }

  Term *proto_paths = (Term *)get_list_pos(terms, 1);
  if (proto_paths->type != LISP_LIST) {
    fprintf(stderr, "Error[pb-setup]: proto paths type is not a LISP_LIST\n");
    ACTION_FAIL();
  }

  size_t num_paths = 0;
  for (L_List p_l = proto_paths->u.l_list; p_l; p_l = l_cdr(p_l)) {
    Term *t = l_car(p_l);
    if (t->type != STRING) {
      fprintf(stderr, "Error[pb-setup]: at least one proto path is not of type STRING\n");
      ACTION_FAIL();
    }
    num_paths += 1;
  }

  std::vector<std::string> ppaths(num_paths);
  size_t i = 0;
  for (L_List p_l = proto_paths->u.l_list; p_l; p_l = l_cdr(p_l)) {
    Term *t = l_car(p_l);
    ppaths[i++] = t->u.string;
  }

  for (size_t i = 0; i < ppaths.size(); ++i) {
    std::string::size_type pos;
    if ((pos = ppaths[i].find("@BASEDIR@")) != std::string::npos) {
      ppaths[i].replace(pos, 9, BASEDIR);
    }
    if ((pos = ppaths[i].find("@FAWKES_BASEDIR@")) != std::string::npos) {
      ppaths[i].replace(pos, 16, FAWKES_BASEDIR);
    }
    if ((pos = ppaths[i].find("@RESDIR@")) != std::string::npos) {
      ppaths[i].replace(pos, 8, RESDIR);
    }
    if ((pos = ppaths[i].find("@CONFDIR@")) != std::string::npos) {
      ppaths[i].replace(pos, 9, CONFDIR);
    }
    if (ppaths[i][ppaths.size()-1] != '/') {
      ppaths[i] += "/";
    }
  }

  delete g_oprs_pb;
  g_oprs_pb = new OpenPRSProtobuf(ppaths);

  ACTION_FINAL();
}


extern "C"
Term *
func_pb_create(TermList terms)
{
  ASSERT_PB;

  Term *type;

  ACTION_ASSERT_ARG_LENGTH("pb-create", terms, 1);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-create", type, terms, 1, STRING);


  try {
    std::shared_ptr<google::protobuf::Message> *msg =
      g_oprs_pb->oprs_create_msg(type->u.string);

    Term *res = MAKE_OBJECT(Term);
    res->type = U_POINTER;
    res->u.u_pointer = msg;
    return res;
  } catch (std::runtime_error &e) {
    fprintf(stderr, "Cannot create message of type %s: %s", type->u.string, e.what());
    return build_nil();
  }
}


extern "C"
Term *
action_pb_client_connect(TermList terms)
{
  ASSERT_PB;

  Term *host, *port;
  ACTION_ASSERT_ARG_LENGTH("pb-client-connect", terms, 2);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-client-connect", host, terms, 1, STRING);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-client-connect", port, terms, 2, LONG_LONG);

  if (port->u.llintval < 0 || port->u.llintval > 65535) {
    fprintf(stderr, "Error[pb-client-connect]: invalid port %lli\n", port->u.llintval);
    ACTION_FAIL();
  }

  return g_oprs_pb->oprs_pb_client_connect(host->u.string, port->u.llintval);
}


extern "C"
Term *
action_pb_enable_server(TermList terms)
{
  ASSERT_PB;

  Term *port;
  ACTION_ASSERT_ARG_LENGTH("pb-enable-server", terms, 1);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-enable-server", port, terms, 1, LONG_LONG);

  if (port->u.llintval < 0 || port->u.llintval > 65535) {
    fprintf(stderr, "Error[pb-enable-server]: invalid port %lli\n", port->u.llintval);
    ACTION_FAIL();
  }

  try {
    g_oprs_pb->oprs_pb_enable_server(port->u.llintval);
    ACTION_FINAL();
  } catch (fawkes::Exception &e) {
    fprintf(stderr, "Error[pb-enable-server]: %s\n", e.what_no_backtrace());
    ACTION_FAIL();
  }
}


extern "C"
Term *
action_pb_disable_server(TermList terms)
{
  ASSERT_PB;

  ACTION_ASSERT_ARG_LENGTH("pb-disable-server", terms, 0);

  try {
    g_oprs_pb->oprs_pb_disable_server();
    ACTION_FINAL();
  } catch (fawkes::Exception &e) {
    fprintf(stderr, "Error[pb-disable-server]: %s\n", e.what_no_backtrace());
    ACTION_FAIL();
  }
}


extern "C"
Term *
action_pb_peer_create(TermList terms)
{
  ASSERT_PB;

  Term *host, *port;
  ACTION_ASSERT_ARG_LENGTH("pb-peer-create", terms, 2);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create", host, terms, 1, STRING);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create", port, terms, 2, LONG_LONG);

  if (port->u.llintval < 0 || port->u.llintval > 65535) {
    fprintf(stderr, "Error[pb-peer-create]: invalid port %lli\n", port->u.llintval);
    ACTION_FAIL();
  }

  return g_oprs_pb->oprs_pb_peer_create(host->u.string, port->u.llintval);
}


extern "C"
Term *
action_pb_peer_create_local(TermList terms)
{
  ASSERT_PB;

  Term *host, *send_port, *recv_port;
  ACTION_ASSERT_ARG_LENGTH("pb-peer-create-local", terms, 3);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-local", host, terms, 1, STRING);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-local", send_port, terms, 2, LONG_LONG);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-local", recv_port, terms, 3, LONG_LONG);

  if (send_port->u.llintval < 0 || send_port->u.llintval > 65535) {
    fprintf(stderr, "Error[pb-peer-create-local]: invalid send port %lli\n", send_port->u.llintval);
    ACTION_FAIL();
  }
  if (recv_port->u.llintval < 0 || recv_port->u.llintval > 65535) {
    fprintf(stderr, "Error[pb-peer-create-local]: invalid recv port %lli\n", recv_port->u.llintval);
    ACTION_FAIL();
  }

  return g_oprs_pb->oprs_pb_peer_create_local(host->u.string,
					      send_port->u.llintval, recv_port->u.llintval);
}


extern "C"
Term *
action_pb_peer_create_crypto(TermList terms)
{
  ASSERT_PB;

  Term *host, *port, *crypto_key, *cipher;
  ACTION_ASSERT_ARG_LENGTH("pb-peer-create-local", terms, 4);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-crypto", host, terms, 1, STRING);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-crypto", port, terms, 2, LONG_LONG);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-crypto", crypto_key, terms, 3, STRING);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-crypto", cipher, terms, 4, STRING);

  if (port->u.llintval < 0 || port->u.llintval > 65535) {
    fprintf(stderr, "Error[pb-peer-create-local]: invalid port %lli\n", port->u.llintval);
    ACTION_FAIL();
  }

  return g_oprs_pb->oprs_pb_peer_create_crypto(host->u.string, port->u.llintval,
					       crypto_key->u.string, cipher->u.string);
}


extern "C"
Term *
action_pb_peer_create_local_crypto(TermList terms)
{
  ASSERT_PB;

  Term *host, *send_port, *recv_port, *crypto_key, *cipher;
  ACTION_ASSERT_ARG_LENGTH("pb-peer-create-local-crypto", terms, 5);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-local-crypto", host, terms, 1, STRING);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-local-crypto", send_port, terms, 2, LONG_LONG);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-local-crypto", recv_port, terms, 3, LONG_LONG);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-local-crypto", crypto_key, terms, 4, STRING);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-create-local-crypto", cipher, terms, 5, STRING);

  if (send_port->u.llintval < 0 || send_port->u.llintval > 65535) {
    fprintf(stderr, "Error[pb-peer-create-local]: invalid send port %lli\n", send_port->u.llintval);
    ACTION_FAIL();
  }
  if (recv_port->u.llintval < 0 || recv_port->u.llintval > 65535) {
    fprintf(stderr, "Error[pb-peer-create-local]: invalid recv port %lli\n", recv_port->u.llintval);
    ACTION_FAIL();
  }

  printf("Creating local peer. %s:%lli %lli %s %s\n",
	 host->u.string, send_port->u.llintval, recv_port->u.llintval, crypto_key->u.string, cipher->u.string);
  return g_oprs_pb->oprs_pb_peer_create_local_crypto(host->u.string, send_port->u.llintval,
						     recv_port->u.llintval,
						     crypto_key->u.string, cipher->u.string);
}


extern "C"
Term *
action_pb_peer_setup_crypto(TermList terms)
{
  ASSERT_PB;

  Term *peer_id, *crypto_key, *cipher;
  ACTION_ASSERT_ARG_LENGTH("pb-peer-setup-crypto", terms, 4);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-setup-crypto", peer_id, terms, 1, LONG_LONG);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-setup-crypto", crypto_key, terms, 2, STRING);
  ACTION_SET_AND_ASSERT_ARG_TYPE("pb-peer-setup-crypto", cipher, terms, 3, STRING);

  try {
    g_oprs_pb->oprs_pb_peer_setup_crypto(peer_id->u.llintval,
					 crypto_key->u.string, cipher->u.string);
    ACTION_FINAL();
  } catch (fawkes::Exception &e) {
    fprintf(stderr, "Error[pb-peer-setup-crypto]: %s\n", e.what_no_backtrace());
    ACTION_FAIL();
  }
}


extern "C"
PBoolean
pred_pb_events_pending(TermList terms)
{
  ASSERT_B_PB;

  return g_oprs_pb->oprs_pb_events_pending();
}


extern "C"
Term *
action_pb_process(TermList terms)
{
  ASSERT_PB;

  try {
    g_oprs_pb->oprs_pb_process();
    ACTION_FINAL();
  } catch (fawkes::Exception &e) {
    fprintf(stderr, "Error[pb-process]: %s\n", e.what_no_backtrace());
    ACTION_FAIL();
  }
}


#define PB_FIELD_ACCESSOR_FUNC(func_name, print_name)			\
  extern "C"								\
  Term *								\
  func_pb_ ## func_name (TermList terms)				\
  {									\
    ASSERT_PB;								\
									\
    Term *msg, *field_name;						\
    ACTION_ASSERT_ARG_LENGTH(print_name, terms, 2);			\
    ACTION_SET_AND_ASSERT_ARG_TYPE(print_name, msg, terms, 1, U_POINTER); \
    ACTION_SET_AND_ASSERT_ARG_TYPE(print_name, field_name, terms, 2, STRING); \
									\
    return g_oprs_pb->oprs_pb_## func_name (msg->u.u_pointer, field_name->u.string); \
  }

#define PB_FIELD_ACCESSOR_PRED(func_name, print_name)			\
  extern "C"								\
  PBoolean								\
  pred_pb_ ## func_name (TermList terms)				\
  {									\
    ASSERT_B_PB;							\
									\
    Term *msg, *field_name;						\
    ACTION_ASSERT_B_ARG_LENGTH(print_name, terms, 2);		\
    ACTION_SET_AND_ASSERT_B_ARG_TYPE(print_name, msg, terms, 1, U_POINTER);	\
    ACTION_SET_AND_ASSERT_B_ARG_TYPE(print_name, field_name, terms, 2, STRING); \
									\
    return g_oprs_pb->oprs_pb_ ## func_name (msg->u.u_pointer, field_name->u.string); \
  }


#define PB_MESSAGE_FUNC(func_name, print_name)				\
  extern "C"								\
  Term *								\
  func_pb_ ## func_name (TermList terms)				\
  {									\
    ASSERT_PB;								\
    									\
    Term *msg;								\
    ACTION_ASSERT_ARG_LENGTH(print_name, terms, 1);			\
    ACTION_SET_AND_ASSERT_ARG_TYPE(print_name, msg, terms, 1, U_POINTER); \
    									\
    return g_oprs_pb->oprs_pb_ ## func_name (msg->u.u_pointer);		\
  }

#define PB_MESSAGE_ACTION(func_name, print_name)			\
  extern "C"								\
  Term *								\
  action_pb_ ## func_name (TermList terms)				\
  {									\
    ASSERT_PB;								\
									\
    Term *msg;								\
    ACTION_ASSERT_ARG_LENGTH(print_name, terms, 1);			\
    ACTION_SET_AND_ASSERT_ARG_TYPE(print_name, msg, terms, 1, U_POINTER); \
    									\
    return g_oprs_pb->oprs_pb_ ## func_name (msg->u.u_pointer);		\
  }

#define PB_CLIENT_MESSAGE_ACTION(func_name, print_name)			\
  extern "C"								\
  Term *								\
  action_pb_ ## func_name (TermList terms)				\
  {									\
    ASSERT_PB;								\
									\
    Term *client_id, *msg;						\
    ACTION_ASSERT_ARG_LENGTH(print_name, terms, 2);			\
    ACTION_SET_AND_ASSERT_ARG_TYPE(print_name, client_id, terms, 1, LONG_LONG); \
    ACTION_SET_AND_ASSERT_ARG_TYPE(print_name, msg, terms, 2, U_POINTER); \
    									\
    try {								\
      g_oprs_pb->oprs_pb_ ## func_name (client_id->u.llintval,		\
					msg->u.u_pointer);		\
      ACTION_FINAL();							\
    } catch (fawkes::Exception &e) {					\
      fprintf(stderr, "Error[%s]: %s\n", print_name, e.what_no_backtrace()); \
      ACTION_FAIL();							\
    }									\
  }

#define PB_FIELD_SET_ACTION(func_name, print_name)			\
  extern "C"								\
  Term *								\
  action_pb_ ## func_name (TermList terms)				\
  {									\
    ASSERT_PB;								\
    									\
    Term *msg, *field_name, *value;					\
    ACTION_ASSERT_ARG_LENGTH(print_name, terms, 3);			\
    ACTION_SET_AND_ASSERT_ARG_TYPE(print_name, msg, terms, 1, U_POINTER); \
    ACTION_SET_AND_ASSERT_ARG_TYPE(print_name, field_name, terms, 2, STRING); \
    value = (Term *)get_list_pos(terms, 3);				\
    									\
    try {								\
      g_oprs_pb->oprs_pb_## func_name (msg->u.u_pointer, field_name->u.string, value); \
      ACTION_FINAL();							\
    } catch (fawkes::Exception &e) {					\
      fprintf(stderr, "Error[%s]: %s\n", print_name, e.what_no_backtrace()); \
      ACTION_FAIL();							\
    }									\
  }

#define PB_CLIENT_ID_ACTION(func_name, print_name)			\
  extern "C"								\
  Term *								\
  action_pb_ ## func_name (TermList terms)				\
  {									\
    ASSERT_PB;								\
									\
    Term *client_id;							\
    ACTION_ASSERT_ARG_LENGTH(print_name, terms, 1);			\
    ACTION_SET_AND_ASSERT_ARG_TYPE(print_name, client_id, terms, 1, LONG_LONG);	\
									\
    try {								\
      g_oprs_pb->oprs_pb_ ## func_name (client_id->u.llintval);		\
      ACTION_FINAL();							\
    } catch (fawkes::Exception &e) {					\
      fprintf(stderr, "Error[pb-client-connect]: failed on "		\
	      "client %lli\n", client_id->u.llintval);			\
      ACTION_FAIL();							\
    }									\
  }

PB_FIELD_ACCESSOR_FUNC(field_value, "pb-field-value")
PB_FIELD_ACCESSOR_FUNC(field_type,  "pb-field-type")
PB_FIELD_ACCESSOR_FUNC(field_label, "pb-field-label")
PB_FIELD_ACCESSOR_FUNC(field_list,  "pb-field-list")

PB_FIELD_ACCESSOR_PRED(has_field,     "pb-has-field")
PB_FIELD_ACCESSOR_PRED(field_is_list, "pb-field-is-list")

PB_MESSAGE_FUNC(field_names, "pb-field-names")
PB_MESSAGE_FUNC(ref, "pb-ref")
PB_MESSAGE_ACTION(destroy, "pb-destroy")
PB_CLIENT_MESSAGE_ACTION(send, "pb-send")
PB_CLIENT_MESSAGE_ACTION(broadcast, "pb-broadcast")

PB_FIELD_SET_ACTION(set_field, "pb-set-field")
PB_FIELD_SET_ACTION(add_list,  "pb-add-list")

PB_CLIENT_ID_ACTION(disconnect, "pb-disconnect")
PB_CLIENT_ID_ACTION(peer_destroy, "pb-peer-destroy")


/** Entry function for the OpenPRS module. */
extern "C"
void init()
{
  printf("*** LOADING mod_protobuf\n");
  printf("Make sure your kernel calls pb-setup!\n");

  make_and_declare_action("pb-setup", action_pb_setup, 1);
  make_and_declare_action("pb-process", action_pb_process, 0);
  make_and_declare_action("pb-destroy", action_pb_destroy, 1);
  make_and_declare_action("pb-set-field", action_pb_set_field, 3);
  make_and_declare_action("pb-add-list", action_pb_add_list, 3);
  make_and_declare_action("pb-disconnect", action_pb_disconnect, 1);
  make_and_declare_action("pb-peer-destroy", action_pb_peer_destroy, 1);
  make_and_declare_action("pb-send", action_pb_send, 2);
  make_and_declare_action("pb-broadcast", action_pb_broadcast, 2);
  make_and_declare_action("pb-client-connect", action_pb_client_connect, 2);
  make_and_declare_action("pb-enable-server", action_pb_enable_server, 1);
  make_and_declare_action("pb-disable-server", action_pb_enable_server, 1);
  make_and_declare_action("pb-peer-create", action_pb_peer_create, 2);
  make_and_declare_action("pb-peer-create-local", action_pb_peer_create_local, 3);
  make_and_declare_action("pb-peer-create-crypto", action_pb_peer_create_crypto, 4);
  make_and_declare_action("pb-peer-create-local-crypto", action_pb_peer_create_local_crypto, 5);
  make_and_declare_action("pb-peer-setup-crypto", action_pb_peer_setup_crypto, 3);

  make_and_declare_eval_funct("pb-create", func_pb_create, 1);
  make_and_declare_eval_funct("pb-field-names", func_pb_field_names, 1);
  make_and_declare_eval_funct("pb-ref", func_pb_ref, 1);
  make_and_declare_eval_funct("pb-field-value", func_pb_field_value, 2);
  make_and_declare_eval_funct("pb-field-type",  func_pb_field_type,  2);
  make_and_declare_eval_funct("pb-field-label", func_pb_field_label, 2);
  make_and_declare_eval_funct("pb-field-list",  func_pb_field_list,  2);

  make_and_declare_eval_pred("pb-has-field", pred_pb_has_field, 2, TRUE);
  make_and_declare_eval_pred("pb-is-list", pred_pb_field_is_list, 2, TRUE);
  make_and_declare_eval_pred("pb-events-pending", pred_pb_events_pending, 0, TRUE);

  add_user_end_kernel_hook(finalize);
}

/** Finalization function for the OpenPRS module. */
extern "C"
void finalize()
{
  printf("*** DESTROYING mod_protobuf\n");
  delete g_oprs_pb;
  g_oprs_pb = NULL;
}
