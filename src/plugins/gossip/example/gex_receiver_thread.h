
/***************************************************************************
 *  gex_receiver_thread.h - Gossip Example Plugin - Receiver
 *
 *  Created: Thu Mar 06 10:39:53 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_GOSSIP_EXAMPLE_GEX_RECEIVER_THREAD_H_
#define __PLUGINS_GOSSIP_EXAMPLE_GEX_RECEIVER_THREAD_H_

#include <plugins/gossip/aspect/gossip.h>
#include <plugins/gossip/gossip/gossip_group.h>

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/blocked_timing.h>

class GossipExampleReceiverThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GossipAspect
{
 public:
  GossipExampleReceiverThread();
  virtual ~GossipExampleReceiverThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void handle_peer_msg(boost::asio::ip::udp::endpoint &endpoint,
		       uint16_t component_id, uint16_t msg_type,
		       std::shared_ptr<google::protobuf::Message> msg);
  void handle_peer_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg);
  void handle_peer_send_error(std::string msg);

 private:
  boost::signals2::connection sig_rcvd_conn_;
  boost::signals2::connection sig_recv_error_conn_;
  boost::signals2::connection sig_send_error_conn_;
};

#endif
