
/***************************************************************************
 *  clips_ros_thread.cpp -  ROS integration for CLIPS
 *
 *  Created: Tue Oct 22 18:14:41 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include "clips_ros_thread.h"

#include <core/threading/mutex_locker.h>
#include <ros/this_node.h>
#include <ros/master.h>
#include <ros/network.h>
#include <ros/xmlrpc_manager.h>
#ifdef HAVE_NEW_ROS_XMLRPCPP_PATH
#  include <xmlrpcpp/XmlRpc.h>
#else
#  include <XmlRpc.h>
#endif

#include <tuple>

using namespace fawkes;

/** @class ClipsROSThread "clips_ros_thread.h"
 * ROS integration for CLIPS.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsROSThread::ClipsROSThread()
  : Thread("ClipsROSThread", Thread::OPMODE_WAITFORWAKEUP),
    CLIPSFeature("ros"), CLIPSFeatureAspect(this)
{
}


/** Destructor. */
ClipsROSThread::~ClipsROSThread()
{
}


void
ClipsROSThread::init()
{
}


void
ClipsROSThread::finalize()
{
}


void
ClipsROSThread::clips_context_init(const std::string &env_name,
				   LockPtr<CLIPS::Environment> &clips)
{
  envs_[env_name] = clips;

  clips->add_function("ros-get-nodes",
    sigc::slot<void>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsROSThread::clips_ros_get_nodes),
	env_name)
    )
  );

  clips->add_function("ros-get-topics",
    sigc::slot<void>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsROSThread::clips_ros_get_topics),
	env_name)
    )
  );

  clips->add_function("ros-get-topic-connections",
    sigc::slot<void>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsROSThread::clips_ros_get_topic_connections),
	env_name)
    )
  );

  clips->batch_evaluate(SRCDIR"/clips/ros.clp");
}

void
ClipsROSThread::clips_context_destroyed(const std::string &env_name)
{
  envs_.erase(env_name);
}

void
ClipsROSThread::loop()
{
}


void
ClipsROSThread::clips_ros_get_nodes(std::string env_name)
{
  if (envs_.find(env_name) == envs_.end()) {
    logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
		     "Cannot get ROS nodes for environment %s "
		     "(not defined)", env_name.c_str());
    return;
  }

  LockPtr<CLIPS::Environment> &clips = envs_[env_name];

  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();

  if (!ros::master::execute("getSystemState", args, result, payload, true)) {
    logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
		     "Failed to retrieve system state from ROS master");
    return;
  }

  std::map<std::string, RosNodeInfo> nodes;

  // publishers
  for (int j = 0; j < payload[0].size(); ++j) {
    std::string topic = payload[0][j][0];
    XmlRpc::XmlRpcValue val = payload[0][j][1];
    for (int k = 0; k < val.size(); ++k) {
      std::string node_name = payload[0][j][1][k];
      nodes[node_name].published.push_back(topic);
    }
  }

  // subscribers
  for (int j = 0; j < payload[1].size(); ++j) {
    std::string topic = payload[1][j][0];
    XmlRpc::XmlRpcValue val = payload[1][j][1];
    for (int k = 0; k < val.size(); ++k) {
      std::string node_name = payload[1][j][1][k];
      nodes[node_name].subscribed.push_back(topic);
    }
  }

  // services
  for (int j = 0; j < payload[2].size(); ++j) {
    std::string service = payload[2][j][0];
    XmlRpc::XmlRpcValue val = payload[2][j][1];
    for (int k = 0; k < val.size(); ++k) {
      std::string node_name = payload[2][j][1][k];
      nodes[node_name].services.push_back(service);
    }
  }

  fawkes::MutexLocker lock(clips.objmutex_ptr());
  CLIPS::Template::pointer temp = clips->get_template("ros-node");
  if (temp) {
    for (auto n : nodes) {
      CLIPS::Values published, subscribed, services;

      for (auto t : n.second.published)  published.push_back(t);
      for (auto t : n.second.subscribed) subscribed.push_back(t);
      for (auto t : n.second.services)   services.push_back(t);

      CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
      fact->set_slot("name", n.first);
      fact->set_slot("published",  published);
      fact->set_slot("subscribed", subscribed);
      fact->set_slot("services",   services);

      CLIPS::Fact::pointer new_fact = clips->assert_fact(fact);
      if (! new_fact) {
	logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
			 "Failed to assert ros-node fact for %s", n.first.c_str());
      }
    }
  } else {
    logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
		     "Could not get deftemplate 'ros-node'");
  }
}


void
ClipsROSThread::clips_ros_get_topics(std::string env_name)
{
  if (envs_.find(env_name) == envs_.end()) {
    logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
		     "Cannot get ROS nodes for environment %s "
		     "(not defined)", env_name.c_str());
    return;
  }


  LockPtr<CLIPS::Environment> &clips = envs_[env_name];

  ros::master::V_TopicInfo topics;
  if (! ros::master::getTopics(topics)) {
    logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
		     "Failed to retrieve topics ROS master");
    return;
  }

  fawkes::MutexLocker lock(clips.objmutex_ptr());
  CLIPS::Template::pointer temp = clips->get_template("ros-topic");
  if (temp) {
    for (auto t : topics) {
      CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
      fact->set_slot("name", t.name);
      fact->set_slot("type", t.datatype);
      clips->assert_fact(fact);
    }
  } else {
    logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
		     "Could not get deftemplate 'ros-topic'");
  }
}


void
ClipsROSThread::clips_ros_get_topic_connections(std::string env_name)
{
  if (envs_.find(env_name) == envs_.end()) {
    logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
		     "Cannot get bus info for environment %s "
		     "(not defined)", env_name.c_str());
    return;
  }

  LockPtr<CLIPS::Environment> &clips = envs_[env_name];
  fawkes::MutexLocker lock(clips.objmutex_ptr());

  CLIPS::Template::pointer temp = clips->get_template("ros-topic-connection");
  if (! temp) {
    logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
		     "Could not get deftemplate 'ros-topic-connection'");
    return;
  }

  ros::V_string nodes;
  if (! ros::master::getNodes(nodes)) {
    logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
		     "Failed to get nodes from ROS master");
    return;
  }

  std::map<std::string, std::string>    uri_to_node;
  std::map<std::string, XmlRpc::XmlRpcClient *> xmlrpc_clients;

  for (size_t i = 0; i < nodes.size(); ++i) {
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    args[1] = nodes[i];
    if (ros::master::execute("lookupNode", args, result, payload, false)) {
      std::string uri = (std::string)payload;
      uri_to_node[uri] = nodes[i];
      std::string host;
      uint32_t port;
      if (ros::network::splitURI(uri, host, port)) {
	xmlrpc_clients[nodes[i]] = new XmlRpc::XmlRpcClient(host.c_str(), port, "/");
      }
    }
  }

  std::vector<std::tuple<std::string, std::string, std::string> > connections;

  ros::XMLRPCManagerPtr xm = ros::XMLRPCManager::instance();

  for (auto n : xmlrpc_clients) {
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    if (n.second->execute("getBusInfo", args, result)) {

      if (! xm->validateXmlrpcResponse("getBusInfo", result, payload)) {
	logger->log_warn(("CLIPS-ROS|" + env_name).c_str(),
			 "%s returned no valid response on getBusInfo", n.first.c_str());
	continue;
      }

      for (int i = 0; i < payload.size(); ++i) {
	// Array format:
	// ID, destination, direction, transport, topic, connected

	// roscpp does not provide the connected flag, hence regard
	// connections which do not provide it as alive
	bool connected =
	  (payload[i].size() >= 6) ? (bool)payload[i][5] : true;

	std::string topic = payload[i][4];
	std::string from, to;
	std::string nodename = payload[i][1];
	if (uri_to_node.find(nodename) != uri_to_node.end())
	  nodename = uri_to_node[nodename];

	if (std::string(payload[i][2]) == "i") {
	  from = nodename;
	  to   = n.first;
	} else {
	  from = n.first;
	  to   = nodename;
	}

	if (connected) {
	  connections.push_back(make_tuple(topic, from, to));
	}
      }

    } else {
      // node unreachable
      //clips->assert_fact_f("(ros-node-unreachable (name %s))", n.first.c_str());
    }

    delete n.second;
  }

  std::vector<std::tuple<std::string, std::string, std::string> > ::iterator c;
  std::sort(connections.begin(), connections.end());
  c = std::unique(connections.begin(), connections.end());
  connections.resize(c - connections.begin());

  for (auto c : connections) {
    CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
    fact->set_slot("topic", std::get<0>(c));
    fact->set_slot("from",  std::get<1>(c));
    fact->set_slot("to",    std::get<2>(c));
    clips->assert_fact(fact);
  }
}
