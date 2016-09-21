
/***************************************************************************
 *  laser_filter_plugin.cpp - Fawkes Laser Filter Plugin
 *
 *  Created: Sun Mar 13 01:06:51 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
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

#include "laser_filter_plugin.h"

#include "filter_thread.h"

#include <core/threading/barrier.h>
#include <map>
#include <set>
#include <memory>

using namespace fawkes;

/** @class LaserFilterPlugin "laser_filter_plugin.h"
 * Laser filter plugin for Fawkes.
 * This plugin filters laser data. It reads laser data from one or more
 * interfaces, filters it, and writes to an output interface. It supports
 * a virtually arbitrary number of active filters.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
LaserFilterPlugin::LaserFilterPlugin(Configuration *config)
  : Plugin(config)
{
  __barrier = NULL;

  std::set<std::string> configs;
  std::set<std::string> ignored_configs;
  std::map<std::string, LaserFilterThread *> threads;

  std::string prefix = "/plugins/laser-filter/";

  // Read configurations and spawn LaserFilterThreads
#if __cplusplus >= 201103L
  std::unique_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
#else
  std::auto_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
#endif
  while (i->next()) {
    std::string cfg_name = std::string(i->path()).substr(prefix.length());
    cfg_name = cfg_name.substr(0, cfg_name.find("/"));

    if ( (configs.find(cfg_name) == configs.end()) &&
	 (ignored_configs.find(cfg_name) == ignored_configs.end()) ) {

      std::string cfg_prefix = prefix + cfg_name + "/";

      bool active = true;
      try {
	active = config->get_bool((cfg_prefix + "active").c_str());
      } catch (Exception &e) {} // ignored, assume enabled

      try {
	if (active) {
	  LaserFilterThread *thread = new LaserFilterThread(cfg_name, cfg_prefix);
	  thread_list.push_back(thread);
	  threads[cfg_name] = thread;
	  configs.insert(cfg_name);
	} else {
	  //printf("Ignoring laser config %s\n", cfg_name.c_str());
	  ignored_configs.insert(cfg_name);
	}
      } catch(Exception &e) {
	for (ThreadList::iterator i = thread_list.begin();
	     i != thread_list.end(); ++i) {
	  delete *i;
	}
	throw;
      }
    }
  }

  if ( thread_list.empty() ) {
    throw Exception("No active laser filters configured, aborting");
  }

  // Read input and output information for spawned configurations
  // for dependency detection
  std::map<std::string, std::list<std::string> > inputs;
  std::map<std::string, std::list<std::string> > outputs;
  std::set<std::string>::iterator c, d;

  for (c = configs.begin(); c != configs.end(); ++c) {
    std::string cinp = prefix + *c + "/in/";
    std::list<std::string> cinputs;
#if __cplusplus >= 201103L
    std::unique_ptr<Configuration::ValueIterator> in(config->search(cinp.c_str()));
#else
    std::auto_ptr<Configuration::ValueIterator> in(config->search(cinp.c_str()));
#endif
    while (in->next()) {
      if (in->is_string()) {
	cinputs.push_back(in->get_string());
      }
    }

    std::string coutp = prefix + *c + "/out/";
    std::list<std::string> coutputs;
#if __cplusplus >= 201103L
    std::unique_ptr<Configuration::ValueIterator> out(config->search(coutp.c_str()));
#else
    std::auto_ptr<Configuration::ValueIterator> out(config->search(coutp.c_str()));
#endif
    while (out->next()) {
      if (out->is_string()) {
	coutputs.push_back(out->get_string());
      }
    }

    inputs[*c] = cinputs;
    outputs[*c] = coutputs;
  }

  // Detect inter-thread dependencies, setup proper serialization by
  // create a list of threads that one threads depends on and setting
  // it. Setup common "end of filtering" barrier.
  try {
    bool has_deps = false;
    for (c = configs.begin(); c != configs.end(); ++c) {

      //printf("Config %s\n", c->c_str());

      std::list<LaserFilterThread *> depthreads;

      std::list<std::string>::iterator i, o;
      std::list<std::string> &cinputs = inputs[*c];
      for (i = cinputs.begin(); i != cinputs.end(); ++i) {
	//printf("  Input %s\n", i->c_str());

	for (d = configs.begin(); d != configs.end(); ++d) {
	  if (*c == *d)  continue;
	  //printf("    Config %s\n", d->c_str());

	  std::list<std::string> &coutputs = outputs[*d];
	  for (o = coutputs.begin(); o != coutputs.end(); ++o) {
	    //printf("      Output %s\n", o->c_str());
	    if (*i == *o) {
	      has_deps = true;
	      //printf("        *** Dep Thread matches %s for %s\n",
	      //       d->c_str(), o->c_str());
	      depthreads.push_back(threads[*d]);
	      break;
	    }
	  }
	}
      }

      if (! depthreads.empty()) {
	depthreads.sort();
	depthreads.unique();
	threads[*c]->set_wait_threads(depthreads);
      }
    }

    // If any dependencies have been detected, have all threads wait at
    // a common "end of filtering" barrier, which allows for resetting
    // a "need to wait for done" flag.
    if (has_deps) {
      std::map<std::string, LaserFilterThread *>::iterator t;
      __barrier = new Barrier(threads.size());
      for (t = threads.begin(); t != threads.end(); ++t) {
	t->second->set_wait_barrier(__barrier);
      }
    }

  } catch (Exception &e) {
    ThreadList::iterator t;
    for (t = thread_list.begin(); t != thread_list.end(); ++t) {
      delete *t;
    }
    throw;
  }
}


LaserFilterPlugin::~LaserFilterPlugin()
{
  delete __barrier;
}


PLUGIN_DESCRIPTION("Filter laser data in blackboard")
EXPORT_PLUGIN(LaserFilterPlugin)
