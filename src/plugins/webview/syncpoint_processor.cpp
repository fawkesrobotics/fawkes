
/***************************************************************************
 *  syncpoint_processor.cpp - Web request processor for SyncPoints
 *
 *  Created: Thu Aug 14 16:21:42 2014
 *  Copyright  2014  Till Hofmann
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

#include "syncpoint_processor.h"
#include <webview/page_reply.h>
#include <webview/file_reply.h>
#include <webview/error_reply.h>

#include <syncpoint/syncpoint_call_stats.h>
#include <aspect/blocked_timing.h>

#include <string>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <cerrno>

#include <gvc.h>
#include <gvcjob.h>

using namespace fawkes;

/** @class WebviewSyncPointRequestProcessor "tf_processor.h"
 * SyncPoint web request processor.
 * Visualizes SyncPoints.
 * @author Till Hofmann
 */

/** Constructor.
 * @param baseurl Base URL where processor is mounted
 * @param syncpoint_manager SyncPointManager which manages all SyncPoints
 * @param max_age Show only SyncPoint calls which are younger than max_age
 */
WebviewSyncPointRequestProcessor::WebviewSyncPointRequestProcessor(const char *baseurl,
  fawkes::SyncPointManager *syncpoint_manager, float max_age)
{
  baseurl_     = strdup(baseurl);
  baseurl_len_ = strlen(baseurl_);
  syncpoint_manager_ = syncpoint_manager;
  max_age_     = max_age;
}


/** Destructor. */
WebviewSyncPointRequestProcessor::~WebviewSyncPointRequestProcessor()
{
  free(baseurl_);
}


WebReply *
WebviewSyncPointRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if ( strncmp(baseurl_, request->url().c_str(), baseurl_len_) == 0 ) {
    // It is in our URL prefix range
    std::string subpath = request->url().substr(baseurl_len_);

    if (subpath == "/graph.png") {
      std::string graph = all_syncpoints_as_dot(
        syncpoint_manager_->get_syncpoints(), max_age_);

      FILE *f = tmpfile();
      if (NULL == f) {
        return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
            "cannot open tmp file: %s", strerror(errno));
      }
      GVC_t* gvc = gvContext();
      Agraph_t* G = agmemread((char *)graph.c_str());
      gvLayout(gvc, G, (char *)"dot");
      gvRender(gvc, G, "png", f);
      gvFreeLayout(gvc, G);
      agclose(G);
      gvFreeContext(gvc);

      try {
        DynamicFileWebReply *freply = new DynamicFileWebReply(f);
        return freply;
      } catch (fawkes::Exception &e) {
        return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR, *(e.begin()));
      }

    } else {
      WebPageReply *r = new WebPageReply("SyncPoints");
      r->append_body("<p><img src=\"%s/graph.png\" /></p>", baseurl_);
      return r;
    }

    WebPageReply *r = new WebPageReply("SyncPoints");
    r->append_body("<p>Hello World</p>");
    return r;
  } else {
    return NULL;
  }
}

/**
 * Get DOT graph for all SyncPoints
 * @param max_age Show only SyncPoint calls which are younger than max_age
 * @return string representation of DOT graph
 */
std::string
WebviewSyncPointRequestProcessor::all_syncpoints_as_dot(
  const std::set<RefPtr<SyncPoint>,SyncPointSetLessThan> syncpoints,
  float max_age)
{
  std::stringstream graph;
  graph << std::fixed; //fixed point notation
  graph.precision(3); //3 decimal places
  graph << "digraph { graph [fontsize=14]; "
      << "node [fontsize=12]; edge [fontsize=12]; ";
  graph.setf(std::ios::fixed, std::ios::floatfield);

  for (std::set<RefPtr<SyncPoint>, SyncPointSetLessThan>::const_iterator sp_it = syncpoints.begin();
      sp_it != syncpoints.end(); sp_it++) {
    graph << "\"" << (*sp_it)->get_identifier() << "\""
        << " [shape=box];";
  }

  for (std::set<RefPtr<SyncPoint>, SyncPointSetLessThan>::const_iterator sp_it = syncpoints.begin();
      sp_it != syncpoints.end(); sp_it++) {
    graph << "\"" << (*sp_it)->get_identifier() << "\";";

    // EMIT CALLS
    CircularBuffer<SyncPointCall> emit_calls = (*sp_it)->get_emit_calls();
    // generate call stats
    std::map<std::string, SyncPointCallStats> emit_call_stats;
    for (CircularBuffer<SyncPointCall>::iterator emitcalls_it = emit_calls.begin();
        emitcalls_it != emit_calls.end(); emitcalls_it++) {
      // Remove the main thread from the graph, and also remove any emit calls
      // to the SyncPoint "/" to improve the resulting graph.
      // The main thread emits and waits for every hook, which adds a lot of
      // uninteresting edges to the graph.
      // Similarly, all emitters also emit "/", which is not interesting to see.
      if (emitcalls_it->get_caller() == "FawkesMainThread" ||
          (*sp_it)->get_identifier() == "/") {
        continue;
      }
      emit_call_stats[emitcalls_it->get_caller()].update_calls(emitcalls_it->get_call_time());
    }

    for (std::map<std::string, SyncPointCallStats>::iterator emit_call_stats_it = emit_call_stats.begin();
        emit_call_stats_it != emit_call_stats.end(); emit_call_stats_it++) {
      float age = (Time() - emit_call_stats_it->second.get_last_call()).in_sec();
      if (age < max_age) {
      graph << "\"" << emit_call_stats_it->first << "\" -> \""
          <<  (*sp_it)->get_identifier()
          << "\"" << " [label=\""
          << " freq=" << emit_call_stats_it->second.get_call_frequency() << "Hz"
          << " age=" << age << "s"
          << " #calls=" << emit_call_stats_it->second.get_num_calls()
          << "\"" << "];";
      }
    }

    // WAIT FOR ONE CALLS
    CircularBuffer<SyncPointCall> wait_one_calls = (*sp_it)->get_wait_calls(SyncPoint::WAIT_FOR_ONE);
    // generate call stats
    std::map<std::string, SyncPointCallStats> wait_one_call_stats;
    for (CircularBuffer<SyncPointCall>::iterator waitcalls_it = wait_one_calls.begin();
        waitcalls_it != wait_one_calls.end(); waitcalls_it++) {
      wait_one_call_stats[waitcalls_it->get_caller()].update_calls(*waitcalls_it);
    }

    for (std::map<std::string, SyncPointCallStats>::iterator wait_call_stats_it = wait_one_call_stats.begin();
        wait_call_stats_it != wait_one_call_stats.end(); wait_call_stats_it++) {
      if (wait_call_stats_it->first == "FawkesMainThread") {
        continue;
      }
      float age = (Time() - wait_call_stats_it->second.get_last_call()).in_sec();
      if (age < max_age) {
        graph << "\"" << (*sp_it)->get_identifier() << "\"" << " -> "
            << "\"" << wait_call_stats_it->first << "\"" << " [label=" << "\""
            << " avg=" << wait_call_stats_it->second.get_waittime_average() <<  "s"
            << " age=" << age << "s"
            << " #calls=" << wait_call_stats_it->second.get_num_calls()
            //          << " max=" << max_wait_time << "s"
            << "\"";
        if ((*sp_it)->watcher_is_waiting(wait_call_stats_it->first, SyncPoint::WAIT_FOR_ONE)) {
          graph << ",color=\"red\"";
        }
        graph << ",style=dotted";
        graph << "];";
      }
    }

    // WAIT FOR ALL CALLS
    CircularBuffer<SyncPointCall> wait_all_calls = (*sp_it)->get_wait_calls(SyncPoint::WAIT_FOR_ALL);
    // generate call stats
    std::map<std::string, SyncPointCallStats> wait_all_call_stats;
    for (CircularBuffer<SyncPointCall>::iterator waitcalls_it = wait_all_calls.begin();
        waitcalls_it != wait_all_calls.end(); waitcalls_it++) {
      wait_all_call_stats[waitcalls_it->get_caller()].update_calls(*waitcalls_it);
    }

    for (std::map<std::string, SyncPointCallStats>::iterator wait_call_stats_it = wait_all_call_stats.begin();
        wait_call_stats_it != wait_all_call_stats.end(); wait_call_stats_it++) {
      if (wait_call_stats_it->first == "FawkesMainThread") {
        continue;
      }
      float age = (Time() - wait_call_stats_it->second.get_last_call()).in_sec();
      if (age < max_age) {
        graph << "\"" << (*sp_it)->get_identifier() << "\"" << " -> "
            << "\"" << wait_call_stats_it->first << "\"" << " [label=" << "\""
            << " avg=" << wait_call_stats_it->second.get_waittime_average() <<  "s"
            << " age=" << age << "s"
            << " #calls=" << wait_call_stats_it->second.get_num_calls()
            //<< " max=" << max_wait_time << "s"
            << "\"";
        if ((*sp_it)->watcher_is_waiting(wait_call_stats_it->first, SyncPoint::WAIT_FOR_ALL)) {
          graph << ",color=\"red\"";
        }
        graph << ",style=dashed";
        graph << "];";
      }
    }
  }
  // Visualize hook dependencies by directly adding edges from one hook to the
  // next. This is necessary because we removed the main thread from the
  // visualization, which makes sure that the hooks are started in the right
  // order. To retain the dependencies between the hooks, add edges manually.
  // Essentially, these edges represent the order that is guaranteed by the
  // main thread.
  // Note: this expects that
  //   (1) pre loop is the first, post loop th last hook,
  //   (2) no custom enum values (e.g. WAKEUP_HOOK_ACT = 5) are specified.
  for (uint i = BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP;
       i != BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP;
       i++) {
    graph << "\""
      << BlockedTimingAspect::blocked_timing_hook_to_end_syncpoint(
        static_cast<BlockedTimingAspect::WakeupHook>(i)) << "\""
      << " -> " << "\""
      << BlockedTimingAspect::blocked_timing_hook_to_start_syncpoint(
        static_cast<BlockedTimingAspect::WakeupHook>(i+1))
      << "\";";
  }
  graph << "}";
  return graph.str();
}
