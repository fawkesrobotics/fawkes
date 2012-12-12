
/***************************************************************************
 *  mongodb_log_plugin.cpp - Fawkes MongoDB Logging Plugin
 *
 *  Created: Wed Dec 08 23:04:33 2010
 *  Copyright  2010-2012  Tim Niemueller [www.niemueller.de]
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

#include "mongodb_log_bb_thread.h"
#include "mongodb_log_image_thread.h"
#include "mongodb_log_pcl_thread.h"
#include "mongodb_log_logger_thread.h"
#include "mongodb_log_tf_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** MongoDB Logging Plugin.
 * This plugin provides logging of BlackBoard data to MongoDB.
 *
 * @author Tim Niemueller
 */
class MongoLogPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  MongoLogPlugin(Configuration *config) : Plugin(config)
  {
    bool enable_bb = true;
    try {
      enable_bb = config->get_bool("/plugins/mongodb-log/enable-blackboard");
    } catch (Exception &e) {}
    if (enable_bb) {
      thread_list.push_back(new MongoLogBlackboardThread());
    }

    bool enable_pcls = true;
    try {
      enable_pcls = config->get_bool("/plugins/mongodb-log/enable-pointclouds");
    } catch (Exception &e) {}
    if (enable_pcls) {
      thread_list.push_back(new MongoLogPointCloudThread());
    }

    bool enable_images = true;
    try {
      enable_images = config->get_bool("/plugins/mongodb-log/enable-images");
    } catch (Exception &e) {}
    if (enable_images) {
      thread_list.push_back(new MongoLogImagesThread());
    }

    bool enable_logger = true;
    try {
      enable_logger = config->get_bool("/plugins/mongodb-log/enable-logger");
    } catch (Exception &e) {}
    if (enable_logger) {
      thread_list.push_back(new MongoLogLoggerThread());
    }

    bool enable_tf = true;
    try {
    enable_tf = config->get_bool("/plugins/mongodb-log/enable-transforms");
    } catch (Exception &e) {}
    if (enable_tf) {
      thread_list.push_back(new MongoLogTransformsThread());
    }

    if (thread_list.empty()) {
      throw Exception("MongoLogPlugin: no logging thread enabled");
    } 

    std::string database = config->get_string("/plugins/mongodb-log/database");
    config->set_string("/plugins/mongorrd/databases/mongodb-log", database);
  }


  ~MongoLogPlugin()
  {
    try {
      config->erase("/plugins/mongorrd/databases/mongodb-log");
    } catch (fawkes::Exception &e) {} // ignore
  }

};


PLUGIN_DESCRIPTION("Logging of BlackBoard data to MongoDB")
EXPORT_PLUGIN(MongoLogPlugin)
