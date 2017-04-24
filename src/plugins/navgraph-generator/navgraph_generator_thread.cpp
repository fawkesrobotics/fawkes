/***************************************************************************
 *  navgraph_generator_thread.cpp - Plugin to generate navgraphs
 *
 *  Created: Mon Feb 09 17:37:30 2015
 *  Copyright  2015-2017  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_generator_thread.h"
#ifdef HAVE_VISUALIZATION
#  include "visualization_thread.h"
#endif

#include <core/threading/mutex_locker.h>
#include <navgraph/generators/voronoi.h>
#include <navgraph/generators/grid.h>
#include <navgraph/yaml_navgraph.h>
#include <plugins/laser-lines/line_func.h>
#include <plugins/amcl/amcl_utils.h>
#include <utils/misc/string_split.h>


using namespace fawkes;

#define CFG_PREFIX "/navgraph-generator/"

/** @class NavGraphGeneratorThread "navgraph_generator_thread.h"
 * Thread to perform graph-based path planning.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphGeneratorThread::NavGraphGeneratorThread()
  : Thread("NavGraphGeneratorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("NavGraphGeneratorThread")
{
#ifdef HAVE_VISUALIZATION
  vt_ = NULL;
#endif
}

#ifdef HAVE_VISUALIZATION
/** Constructor. */
NavGraphGeneratorThread::NavGraphGeneratorThread(NavGraphGeneratorVisualizationThread *vt)
  : Thread("NavGraphGeneratorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("NavGraphGeneratorThread")
{
  vt_ = vt;
}
#endif


/** Destructor. */
NavGraphGeneratorThread::~NavGraphGeneratorThread()
{
}

void
NavGraphGeneratorThread::init()
{
  bbox_set_ = false;
  copy_default_properties_ = true;

  filter_["FILTER_EDGES_BY_MAP"] = false;
  filter_["FILTER_ORPHAN_NODES"] = false;
  filter_["FILTER_MULTI_GRAPH"] = false;

  filter_params_float_defaults_["FILTER_EDGES_BY_MAP"]["distance"] = 0.3;
  if (config->exists(CFG_PREFIX"filters/edges_by_map/distance")) {
	  filter_params_float_defaults_["FILTER_EDGES_BY_MAP"]["distance"] =
		  config->get_float(CFG_PREFIX"filters/edges_by_map/distance");
  }

  filter_params_float_ = filter_params_float_defaults_;

  cfg_map_line_segm_max_iterations_ =
    config->get_uint(CFG_PREFIX"map/line_segmentation_max_iterations");
  cfg_map_line_segm_min_inliers_ =
    config->get_uint(CFG_PREFIX"map/line_segmentation_min_inliers");
  cfg_map_line_min_length_ =
    config->get_float(CFG_PREFIX"map/line_min_length");
  cfg_map_line_cluster_tolerance_ =
    config->get_float(CFG_PREFIX"map/line_cluster_tolerance");
  cfg_map_line_cluster_quota_ =
    config->get_float(CFG_PREFIX"map/line_cluster_quota");

  cfg_global_frame_  = config->get_string("/frames/fixed");

  cfg_visualization_ = false;
  try {
    cfg_visualization_ = config->get_bool(CFG_PREFIX"visualization/enable");
  } catch (Exception &e) {} // ignore, use default

  cfg_save_to_file_ = false;
  try {
    cfg_save_to_file_ = config->get_bool(CFG_PREFIX"save-to-file/enable");
  } catch (Exception &e) {} // ignore, use default
  if (cfg_save_to_file_) {
	  cfg_save_filename_ = config->get_string(CFG_PREFIX"save-to-file/filename");
	  if (cfg_save_filename_.empty()) {
		  throw Exception("navgraph-generator: invalid empty filename");
	  }
	  if (cfg_save_filename_.find("..") != std::string::npos) {
		  throw Exception("navgraph-generator: filename may not contains two consecutive dots (..)");
	  }
	  if (cfg_save_filename_[0] != '/') {
		  cfg_save_filename_ = std::string(CONFDIR) + "/" + cfg_save_filename_;
	  }
  }

#ifndef HAVE_VISUALIZATION
  if (cfg_visualization_) {
    logger->log_warn(name(), "Visualization enabled, but support not compiled in");
  }
#endif

  navgen_if_ =
    blackboard->open_for_writing<NavGraphGeneratorInterface>("/navgraph-generator");
  bbil_add_message_interface(navgen_if_);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);

}

void
NavGraphGeneratorThread::finalize()
{
  blackboard->unregister_listener(this);
  bbil_remove_message_interface(navgen_if_);
  blackboard->close(navgen_if_);
}


void
NavGraphGeneratorThread::loop()
{
	std::shared_ptr<NavGraphGenerator> ng;

	try {
		switch (algorithm_) {
		case fawkes::NavGraphGeneratorInterface::ALGORITHM_GRID:
			ng.reset(new NavGraphGeneratorGrid(algorithm_params_));
			break;
		default:
			ng.reset(new NavGraphGeneratorVoronoi());
		}
	} catch (Exception &e) {
		logger->log_error(name(), "Failed to initialize algorithm %s, exception follows",
		                  navgen_if_->tostring_Algorithm(algorithm_));
		logger->log_error(name(), e);
		navgen_if_->set_ok(false);
		navgen_if_->set_error_message(e.what_no_backtrace());
		navgen_if_->set_final(true);
		navgen_if_->write();
		return;
	}

	logger->log_debug(name(), "Calculating new graph (%s)", navgen_if_->tostring_Algorithm(algorithm_));

  if (bbox_set_) {
    logger->log_debug(name(), "  Setting bound box (%f,%f) to (%f,%f)",
		      bbox_p1_.x, bbox_p1_.y, bbox_p2_.x, bbox_p2_.y);
    ng->set_bounding_box(bbox_p1_.x, bbox_p1_.y, bbox_p2_.x, bbox_p2_.y);
  }

  for (auto o : obstacles_) {
    logger->log_debug(name(), "  Adding obstacle %s at (%f,%f)",
		      o.first.c_str(), o.second.x, o.second.y);
    ng->add_obstacle(o.second.x, o.second.y);
  }

  for (auto o : map_obstacles_) {
    logger->log_debug(name(), "  Adding map obstacle %s at (%f,%f)",
		      o.first.c_str(), o.second.x, o.second.y);
    ng->add_obstacle(o.second.x, o.second.y);
  }

  // Acquire lock on navgraph, no more searches/modifications until we are done
  MutexLocker lock(navgraph.objmutex_ptr());

  // disable notifications as to not trigger one for all the many
  // operations we are going to perform
  navgraph->set_notifications_enabled(false);

  // remember default properties
  std::map<std::string, std::string> default_props = navgraph->default_properties();

  navgraph->clear();

  // restore default properties
  if (copy_default_properties_) {
    navgraph->set_default_properties(default_props);
  }

  // set properties received as message
  for (auto p : default_properties_) {
    navgraph->set_default_property(p.first, p.second);
  }

  logger->log_debug(name(), "  Computing navgraph");
  try {
	  ng->compute(navgraph);
  } catch (Exception &e) {
	  logger->log_error(name(), "Failed to compute navgraph, exception follows");
	  logger->log_error(name(), e);
	  navgen_if_->set_ok(false);
	  navgen_if_->set_error_message(e.what_no_backtrace());
	  navgen_if_->write();
	  navgraph->set_notifications_enabled(true);
	  return;
  }

  // post-processing
  if (filter_["FILTER_EDGES_BY_MAP"]) {
    logger->log_debug(name(), "  Applying FILTER_EDGES_BY_MAP");
    filter_edges_from_map(filter_params_float_["FILTER_EDGES_BY_MAP"]["distance"]);
  }
  if (filter_["FILTER_ORPHAN_NODES"]) {
    logger->log_debug(name(), "  Applying FILTER_ORPHAN_NODES");
    filter_nodes_orphans();
  }
  if (filter_["FILTER_MULTI_GRAPH"]) {
    logger->log_debug(name(), "  Applying FILTER_MULTI_GRAPH");
    filter_multi_graph();
  }

  // add POIs
  for (const auto &p : pois_) {
    // add poi
	  NavGraphNode node(p.first, p.second.position.x, p.second.position.y,
	                    p.second.properties);
    switch (p.second.conn_mode) {
    case NavGraphGeneratorInterface::NOT_CONNECTED:
	    logger->log_debug(name(), "  POI without initial connection %s at (%f,%f)",
                        p.first.c_str(), p.second.position.x, p.second.position.y);
      navgraph->add_node(node);
      break;

    case NavGraphGeneratorInterface::UNCONNECTED:
	    logger->log_debug(name(), "  Unconnected POI %s at (%f,%f)",
	                      p.first.c_str(), p.second.position.x, p.second.position.y);
	    node.set_unconnected(true);
      navgraph->add_node(node);
      break;

    case NavGraphGeneratorInterface::CLOSEST_NODE:
	    logger->log_debug(name(), "  Connecting POI %s at (%f,%f) to closest node",
                        p.first.c_str(), p.second.position.x, p.second.position.y);
      navgraph->add_node_and_connect(node, NavGraph::CLOSEST_NODE);
      break;
    case NavGraphGeneratorInterface::CLOSEST_EDGE:
      try {
	      logger->log_debug(name(), "  Connecting POI %s at (%f,%f) to closest edge",
	                        p.first.c_str(), p.second.position.x, p.second.position.y);
	      navgraph->add_node_and_connect(node, NavGraph::CLOSEST_EDGE);
      } catch (Exception &e) {
	      logger->log_error(name(), "  Failed to add POI %s, exception follows",
	                        p.first.c_str());
	      logger->log_error(name(), e);
      }
      break;
    case NavGraphGeneratorInterface::CLOSEST_EDGE_OR_NODE:
	    logger->log_debug(name(), "  Connecting POI %s at (%f,%f) to closest edge or node",
	                      p.first.c_str(), p.second.position.x, p.second.position.y);
	    navgraph->add_node_and_connect(node, NavGraph::CLOSEST_EDGE_OR_NODE);
	    break;
    }
  }


  // add edges
  for (const auto &e : edges_) {
    switch (e.edge_mode) {
    case NavGraphGeneratorInterface::NO_INTERSECTION:
	    logger->log_debug(name(), "  Edge %s-%s%s (no intersection)",
	                      e.p1.c_str(), e.directed ? ">" : "-", e.p2.c_str());
	    navgraph->add_edge(NavGraphEdge(e.p1, e.p2, e.directed),
	                       NavGraph::EDGE_NO_INTERSECTION);
      break;

    case NavGraphGeneratorInterface::SPLIT_INTERSECTION:
	    logger->log_debug(name(), "  Edge %s-%s%s (split intersection)",
	                      e.p1.c_str(), e.directed ? ">" : "-", e.p2.c_str());
      navgraph->add_edge(NavGraphEdge(e.p1, e.p2, e.directed),
                         NavGraph::EDGE_SPLIT_INTERSECTION);
      break;

    case NavGraphGeneratorInterface::FORCE:
	    logger->log_debug(name(), "  Edge %s-%s%s (force)",
	                      e.p1.c_str(), e.directed ? ">" : "-", e.p2.c_str());
	    navgraph->add_edge(NavGraphEdge(e.p1, e.p2, e.directed),
	                       NavGraph::EDGE_FORCE);
      break;
    }
  }

  /*
	// Add POIs in free areas
	unsigned int ci = 0;
	const std::list<Polygon2D> &polygons = nggv.face_polygons();
	for (const auto &p : polygons) {
		Eigen::Vector2f centroid(polygon_centroid(p));
		navgraph->add_node_and_connect(NavGraphNode(navgraph->format_name("AREA-%u", ++ci),
		                                            centroid.x(), centroid.y()),
		                               NavGraph::CLOSEST_EDGE_OR_NODE);
		
	}
  */

  // Finalize graph setup
  try {
    logger->log_debug(name(), "  Calculate reachability relations");
    navgraph->calc_reachability();
  } catch (Exception &e) {
	  logger->log_error(name(), "Failed to finalize graph setup, exception follows");
	  logger->log_error(name(), e);
  }

  if (cfg_save_to_file_) {
	  logger->log_debug(name(), "  Writing to file '%s'", cfg_save_filename_.c_str());
	  save_yaml_navgraph(cfg_save_filename_, *navgraph);
  }

  // re-enable notifications
  navgraph->set_notifications_enabled(true);

  logger->log_debug(name(), "  Graph computed, notifying listeners");
  navgraph->notify_of_change();

  navgen_if_->set_ok(true);
  navgen_if_->set_final(true);
  navgen_if_->write();

#ifdef HAVE_VISUALIZATION
  if (cfg_visualization_)  publish_visualization();
#endif
}


bool
NavGraphGeneratorThread::bb_interface_message_received(Interface *interface,
                                                       Message *message) throw()
{
  // in continuous mode wait for signal if disabled
  MutexLocker lock(loop_mutex);

  if (message->is_of_type<NavGraphGeneratorInterface::ClearMessage>()) {
    pois_.clear();
    obstacles_.clear();
    map_obstacles_.clear();
    edges_.clear();
    default_properties_.clear();
    bbox_set_ = false;
    copy_default_properties_ = true;
    algorithm_ = fawkes::NavGraphGeneratorInterface::ALGORITHM_VORONOI;
    algorithm_params_.clear();
    filter_params_float_ = filter_params_float_defaults_;
    for (auto &f : filter_) {
      f.second = false;
    }

  } else if (message->is_of_type<NavGraphGeneratorInterface::SetAlgorithmMessage>()) {
	  NavGraphGeneratorInterface::SetAlgorithmMessage *msg =
		  message->as_type<NavGraphGeneratorInterface::SetAlgorithmMessage>();

	  algorithm_ = msg->algorithm();

  } else if (message->is_of_type<NavGraphGeneratorInterface::SetAlgorithmParameterMessage>()) {
    NavGraphGeneratorInterface::SetAlgorithmParameterMessage *msg =
      message->as_type<NavGraphGeneratorInterface::SetAlgorithmParameterMessage>();

    algorithm_params_[msg->param()] = msg->value();

  } else if (message->is_of_type<NavGraphGeneratorInterface::SetBoundingBoxMessage>()) {
    NavGraphGeneratorInterface::SetBoundingBoxMessage *msg =
      message->as_type<NavGraphGeneratorInterface::SetBoundingBoxMessage>();
    bbox_set_  = true;
    bbox_p1_.x = msg->p1_x();
    bbox_p1_.y = msg->p1_y();
    bbox_p2_.x = msg->p2_x();
    bbox_p2_.y = msg->p2_y();

  } else if (message->is_of_type<NavGraphGeneratorInterface::SetFilterMessage>()) {
    NavGraphGeneratorInterface::SetFilterMessage *msg =
      message->as_type<NavGraphGeneratorInterface::SetFilterMessage>();

    filter_[navgen_if_->tostring_FilterType(msg->filter())] = msg->is_enable();

  } else if (message->is_of_type<NavGraphGeneratorInterface::SetFilterParamFloatMessage>()) {
    NavGraphGeneratorInterface::SetFilterParamFloatMessage *msg =
      message->as_type<NavGraphGeneratorInterface::SetFilterParamFloatMessage>();

    std::map<std::string, float> &param_float =
      filter_params_float_[navgen_if_->tostring_FilterType(msg->filter())];

    if (param_float.find(msg->param()) != param_float.end()) {
      param_float[msg->param()] = msg->value();
    } else {
      logger->log_warn(name(), "Filter %s has no float parameter named %s, ignoring",
		       navgen_if_->tostring_FilterType(msg->filter()),
		       msg->param());
    }

  } else if (message->is_of_type<NavGraphGeneratorInterface::AddMapObstaclesMessage>()) {
    NavGraphGeneratorInterface::AddMapObstaclesMessage *msg =
      message->as_type<NavGraphGeneratorInterface::AddMapObstaclesMessage>();
    map_obstacles_ = map_obstacles(msg->max_line_point_distance());

  } else if (message->is_of_type<NavGraphGeneratorInterface::AddObstacleMessage>()) {
    NavGraphGeneratorInterface::AddObstacleMessage *msg =
      message->as_type<NavGraphGeneratorInterface::AddObstacleMessage>();
    if (std::isfinite(msg->x()) && std::isfinite(msg->x())) {
      obstacles_[msg->name()] = cart_coord_2d_t(msg->x(), msg->y());
    } else {
      logger->log_error(name(), "Received non-finite obstacle (%.2f,%.2f), ignoring",
			msg->x(), msg->y());
    }
  } else if (message->is_of_type<NavGraphGeneratorInterface::RemoveObstacleMessage>()) {
    NavGraphGeneratorInterface::RemoveObstacleMessage *msg =
      message->as_type<NavGraphGeneratorInterface::RemoveObstacleMessage>();
    ObstacleMap::iterator f;
    if ((f = obstacles_.find(msg->name())) != obstacles_.end()) {
      obstacles_.erase(f);
    }

  } else if (message->is_of_type<NavGraphGeneratorInterface::AddPointOfInterestMessage>()) {
    NavGraphGeneratorInterface::AddPointOfInterestMessage *msg =
      message->as_type<NavGraphGeneratorInterface::AddPointOfInterestMessage>();
    if (std::isfinite(msg->x()) && std::isfinite(msg->x())) {
      PointOfInterest poi;
      poi.position  = cart_coord_2d_t(msg->x(), msg->y());
      poi.conn_mode = msg->mode();
      pois_[msg->name()] = poi;
    } else {
      logger->log_error(name(), "Received non-finite POI (%.2f,%.2f), ignoring",
			msg->x(), msg->y());
    }

  } else if (message->is_of_type<NavGraphGeneratorInterface::AddEdgeMessage>()) {
    NavGraphGeneratorInterface::AddEdgeMessage *msg =
      message->as_type<NavGraphGeneratorInterface::AddEdgeMessage>();
    Edge edge;
    edge.p1 = msg->p1();
    edge.p2 = msg->p2();
    edge.directed = msg->is_directed();
    edge.edge_mode = msg->mode();
    edges_.push_back(edge);

  } else if (message->is_of_type<NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage>()) {
    NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage *msg =
      message->as_type<NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage>();

    if (std::isfinite(msg->x()) && std::isfinite(msg->x())) {
      PointOfInterest poi;
      poi.position  = cart_coord_2d_t(msg->x(), msg->y());
      poi.conn_mode = msg->mode();
      if (std::isfinite(msg->ori())) {
	poi.properties["orientation"] = std::to_string(msg->ori());
      } else {
	logger->log_warn(name(), "Received POI with non-finite ori %f, ignoring ori",
			 msg->ori());
      }
      pois_[msg->name()] = poi;
    } else {
      logger->log_error(name(), "Received non-finite POI (%.2f,%.2f), ignoring ori",
			msg->x(), msg->y());
    }

  } else if (message->is_of_type<NavGraphGeneratorInterface::RemovePointOfInterestMessage>()) {
    NavGraphGeneratorInterface::RemovePointOfInterestMessage *msg =
      message->as_type<NavGraphGeneratorInterface::RemovePointOfInterestMessage>();
    PoiMap::iterator f;
    if ((f = pois_.find(msg->name())) != pois_.end()) {
      pois_.erase(f);
    }

  } else if (message->is_of_type<NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage>()) {
    NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage *msg =
      message->as_type<NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage>();
    PoiMap::iterator f;
    if ((f = pois_.find(msg->name())) != pois_.end()) {
      f->second.properties[msg->property_name()] = msg->property_value();
    } else {
      logger->log_warn(name(), "POI %s unknown, cannot set property %s=%s",
		       msg->name(), msg->property_name(), msg->property_value());
    }

  } else if (message->is_of_type<NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage>()) {
    NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage *msg =
      message->as_type<NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage>();
    default_properties_[msg->property_name()] = msg->property_value();

  } else if (message->is_of_type<NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage>()) {
    NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage *msg =
      message->as_type<NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage>();
    copy_default_properties_ = msg->is_enable_copy();

  } else if (message->is_of_type<NavGraphGeneratorInterface::ComputeMessage>()) {
    navgen_if_->set_msgid(message->id());
    navgen_if_->set_final(false);
    navgen_if_->write();
    wakeup();

  } else {
    logger->log_error(name(), "Received unknown message of type %s, ignoring",
		      message->type());
  }

  return false;
}


map_t *
NavGraphGeneratorThread::load_map(std::vector<std::pair<int, int>> &free_space_indices)
{
  std::string  cfg_map_file;
  float        cfg_resolution;
  float        cfg_origin_x;
  float        cfg_origin_y;
  float        cfg_origin_theta;
  float        cfg_occupied_thresh;
  float        cfg_free_thresh;

  fawkes::amcl::read_map_config(config, cfg_map_file, cfg_resolution, cfg_origin_x,
				cfg_origin_y, cfg_origin_theta, cfg_occupied_thresh,
				cfg_free_thresh);

  return fawkes::amcl::read_map(cfg_map_file.c_str(),
				cfg_origin_x, cfg_origin_y, cfg_resolution,
				cfg_occupied_thresh, cfg_free_thresh, free_space_indices);
}

NavGraphGeneratorThread::ObstacleMap
NavGraphGeneratorThread::map_obstacles(float line_max_dist)
{
  ObstacleMap  obstacles;
  unsigned int obstacle_i = 0;

  std::vector<std::pair<int, int> > free_space_indices;
  map_t *map = load_map(free_space_indices);

  logger->log_info(name(), "Map Obstacles: map size: %ux%u (%zu of %u cells free, %.1f%%)",
                   map->size_x, map->size_y, free_space_indices.size(),
		   map->size_x * map->size_y,
                   (float)free_space_indices.size() / (float)(map->size_x * map->size_y) * 100.);

  size_t occ_cells = map->size_x * map->size_y - free_space_indices.size();

  // convert map to point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  map_cloud->points.resize(occ_cells);
  size_t pi = 0;
  for (int x = 0; x < map->size_x; ++x) {
    for (int y = 0; y < map->size_y; ++y) {
      if (map->cells[MAP_INDEX(map, x, y)].occ_state > 0) {
	// cell is occupied, generate point in cloud
	pcl::PointXYZ p;
	p.x = MAP_WXGX(map, x) + 0.5 * map->scale;
	p.y = MAP_WYGY(map, y) + 0.5 * map->scale;
	p.z = 0.;
	map_cloud->points[pi++] = p;
      }
    }
  }

  logger->log_info(name(), "Map Obstacles: filled %zu/%zu points", pi, occ_cells);

  pcl::PointCloud<pcl::PointXYZ>::Ptr
    no_line_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  // determine lines
  std::vector<LineInfo> linfos =
    calc_lines<pcl::PointXYZ>(map_cloud,
			      cfg_map_line_segm_min_inliers_, cfg_map_line_segm_max_iterations_,
			      /* segm distance threshold */ 2 * map->scale,
			      /* segm sample max dist */  2 * map->scale,
			      cfg_map_line_cluster_tolerance_, cfg_map_line_cluster_quota_,
			      cfg_map_line_min_length_, -1, -1, -1,
			      no_line_cloud);

  logger->log_info(name(), "Map Obstacles: found %zu lines, %zu points remaining",
		   linfos.size(), no_line_cloud->points.size());


  // determine line obstacle points
  for (const LineInfo &line : linfos) {
    const unsigned int num_points = ceilf(line.length / line_max_dist);
    float distribution = line.length / num_points;

    obstacles[NavGraph::format_name("Map_%u", ++obstacle_i)] =
      cart_coord_2d_t(line.end_point_1[0], line.end_point_1[1]);
    for (unsigned int i = 1; i <= num_points; ++i) {
      Eigen::Vector3f p = line.end_point_1 + i * distribution * line.line_direction;
      obstacles[NavGraph::format_name("Map_%d", ++obstacle_i)] =  cart_coord_2d_t(p[0], p[1]);
    }
  }

  // cluster in remaining points to find more points of interest
  pcl::search::KdTree<pcl::PointXYZ>::Ptr
    kdtree_cluster(new pcl::search::KdTree<pcl::PointXYZ>());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(2 * map->scale);
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(no_line_cloud->points.size());
  ec.setSearchMethod(kdtree_cluster);
  ec.setInputCloud(no_line_cloud);
  ec.extract(cluster_indices);

  unsigned int i = 0;
  for (auto cluster : cluster_indices) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*no_line_cloud, cluster.indices, centroid);

    logger->log_info(name(), "Map Obstacles: Cluster %u with %zu points at (%f, %f, %f)",
		     i, cluster.indices.size(), centroid.x(), centroid.y(), centroid.z());

    obstacles[NavGraph::format_name("MapCluster_%u", ++i)] =  cart_coord_2d_t(centroid.x(), centroid.y());
  }

  map_free(map);

  return obstacles;
}


void
NavGraphGeneratorThread::filter_edges_from_map(float max_dist)
{
  std::vector<std::pair<int, int> > free_space_indices;
  map_t *map = load_map(free_space_indices);

  const std::vector<NavGraphEdge> &edges = navgraph->edges();

  for (int x = 0; x < map->size_x; ++x) {
    for (int y = 0; y < map->size_y; ++y) {
      if (map->cells[MAP_INDEX(map, x, y)].occ_state > 0) {
	// cell is occupied, generate point in cloud
	Eigen::Vector2f gp;
	gp[0] = MAP_WXGX(map, x) + 0.5 * map->scale;
	gp[1] = MAP_WYGY(map, y) + 0.5 * map->scale;

	for (const NavGraphEdge &e : edges) {
	  try {
	    cart_coord_2d_t poe = e.closest_point_on_edge(gp[0], gp[1]);
	    Eigen::Vector2f p;
	    p[0] = poe.x; p[1] = poe.y;
	    if ((gp - p).norm() <= max_dist) {
	      // edge too close, remove it!
	      logger->log_debug(name(),
				"  Removing edge (%s--%s), too close to occupied map cell (%f,%f)",
				e.from().c_str(), e.to().c_str(), gp[0], gp[1]);
	      navgraph->remove_edge(e);
	      break;
	    }
	  } catch (Exception &e) {} // alright, not close
	}
      }
    }
  }
  map_free(map);
}


void
NavGraphGeneratorThread::filter_nodes_orphans()
{
  const std::vector<NavGraphEdge> &edges = navgraph->edges();
  const std::vector<NavGraphNode> &nodes = navgraph->nodes();

  std::list<NavGraphNode> remove_nodes;

  for (const NavGraphNode &n : nodes) {
    std::string nname = n.name();
    std::vector<NavGraphEdge>::const_iterator e =
      std::find_if(edges.begin(), edges.end(),
		   [nname](const NavGraphEdge &e)->bool{
		     return (e.from() == nname || e.to() == nname);
		   });
    if (e == edges.end() && ! n.unconnected()) {
      // node is not connected to any other node -> remove
      remove_nodes.push_back(n);
    }
  }

  for (const NavGraphNode &n : remove_nodes) {
    logger->log_debug(name(), "  Removing unconnected node %s", n.name().c_str());
    navgraph->remove_node(n);
  }
}


void
NavGraphGeneratorThread::filter_multi_graph()
{
  navgraph->calc_reachability(/* allow multi graph*/ true);

  std::list<std::set<std::string>> graphs;

  const std::vector<NavGraphNode> &nodes = navgraph->nodes();
  std::set<std::string> nodeset;
  std::for_each(nodes.begin(), nodes.end(),
		[&nodeset](const NavGraphNode &n){ nodeset.insert(n.name()); });

  while (! nodeset.empty()) {
    std::queue<std::string> q;
    q.push(* nodeset.begin());

    std::set<std::string> traversed;

    while (! q.empty()) {
      std::string &nname = q.front();
      traversed.insert(nname);

      NavGraphNode n = navgraph->node(nname);
      if (n) {
	const std::vector<std::string> & reachable = n.reachable_nodes();

	for (const std::string &r : reachable) {
	  if (traversed.find(r) == traversed.end()) q.push(r);
	}
      }
      q.pop();
    }

    std::set<std::string> nodediff;
    std::set_difference(nodeset.begin(), nodeset.end(),
			traversed.begin(), traversed.end(),
			std::inserter(nodediff, nodediff.begin()));
    graphs.push_back(traversed);
    nodeset = nodediff;
  }

  // reverse sort, largest set first
  graphs.sort([](const std::set<std::string> &a, const std::set<std::string> &b)->bool{
      return b.size() < a.size();   
    });

  std::for_each(std::next(graphs.begin()), graphs.end(),
		[&](const std::set<std::string> &g) {
		  logger->log_debug(name(), "  Removing disconnected sub-graph [%s]",
				    str_join(g.begin(), g.end(), ", ").c_str());
		  for (const std::string &n : g)  navgraph->remove_node(n);
		});
}


#ifdef HAVE_VISUALIZATION
void
NavGraphGeneratorThread::publish_visualization()
{
  if (vt_) {
    vt_->publish(obstacles_, map_obstacles_, pois_);
  }
}
#endif
