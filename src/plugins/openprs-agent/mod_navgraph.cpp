/***************************************************************************
 *  mod_navgraph.cpp -  OpenPRS navgraph module
 *
 *  Created: Fri Sep 05 16:22:26 2014
 *  Copyright  2014-2015  Tim Niemueller [www.niemueller.de]
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
#include <navgraph/yaml_navgraph.h>
#include <navgraph/navgraph.h>
#include <oprs_f-pub.h>

using namespace fawkes;

extern "C" void finalize();

// Global variables
FawkesNetworkClient  *g_fnet_client = NULL;
NetworkConfiguration *g_config = NULL;
NavGraph  *g_navgraph = NULL;


extern "C"
Term *
action_navgraph_load(TermList terms)
{
  ACTION_ASSERT_ARG_LENGTH("navgraph-load", terms, 0);

  try {
    std::string graph_file =
      g_config->get_string("/navgraph/graph_file");

    if (graph_file[0] != '/') {
      graph_file = std::string(CONFDIR) + "/" + graph_file;
    }

    g_navgraph = load_yaml_navgraph(graph_file);

    const std::vector<NavGraphNode> &nodes = g_navgraph->nodes();
    const std::vector<NavGraphEdge> &edges = g_navgraph->edges();

    TermList graph_tl = sl_make_slist();
    graph_tl = build_term_list(graph_tl, build_string(g_navgraph->name().c_str()));
    graph_tl = build_term_list(graph_tl, build_string(graph_file.c_str()));
    add_external_fact((char *)"navgraph", graph_tl);

    for (auto n : nodes) {
      TermList props = sl_make_slist();
      const std::map<std::string, std::string> &properties = n.properties();
      for (auto p : properties) {
	TermList prop = sl_make_slist();
	prop = build_term_list(prop, build_string(p.first.c_str()));
	prop = build_term_list(prop, build_string(p.second.c_str()));
	props = build_term_list(props, build_term_l_list_from_c_list(prop));
      }

      TermList node_tl = sl_make_slist();
      node_tl = build_term_list(node_tl, build_string(n.name().c_str()));
      node_tl = build_term_list(node_tl, build_float(n.x()));
      node_tl = build_term_list(node_tl, build_float(n.y()));
      node_tl = build_term_list(node_tl, build_term_l_list_from_c_list(props));

      add_external_fact((char *)"navgraph-node", node_tl);
    }

    for (auto e : edges) {
      TermList props = sl_make_slist();
      const std::map<std::string, std::string> &properties = e.properties();
      for (auto p : properties) {
	TermList prop = sl_make_slist();
	prop = build_term_list(prop, build_string(p.first.c_str()));
	prop = build_term_list(prop, build_string(p.second.c_str()));
	props = build_term_list(props, build_term_l_list_from_c_list(prop));
      }

      TermList edge_tl = sl_make_slist();
      edge_tl = build_term_list(edge_tl, build_string(e.from().c_str()));
      edge_tl = build_term_list(edge_tl, build_string(e.to().c_str()));
      edge_tl = build_term_list(edge_tl, e.is_directed() ? build_t() : build_nil());
      edge_tl = build_term_list(edge_tl, build_term_l_list_from_c_list(props));

      add_external_fact((char *)"navgraph-edge", edge_tl);
    }

  } catch (Exception &e) {
    fprintf(stderr, "Failed to open navgraph: %s\n", e.what_no_backtrace());
    ACTION_FAIL();
  }

  ACTION_FINAL();
}


/** Entry function for the OpenPRS module. */
extern "C"
void init()
{
  printf("*** LOADING mod_navgraph\n");

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

  make_and_declare_action("navgraph-load", action_navgraph_load, 0);
  add_user_end_kernel_hook(finalize);
}

/** Finalization function for the OpenPRS module. */
extern "C"
void finalize()
{
  printf("*** DESTROYING mod_navgraph\n");
  delete g_config;
  g_config = NULL;
  delete g_fnet_client;
  g_fnet_client = NULL;
  delete g_navgraph;
  g_navgraph = NULL;
}
