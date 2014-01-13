
/***************************************************************************
 *  pcl_db_pipeline.h - PCL DB processing pipeline base class
 *
 *  Created: Wed Aug 21 17:24:18 2013
 *  Copyright  2012-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PERCEPTION_PCL_DB_PCL_DB_PIPELINE_H_
#define __PLUGINS_PERCEPTION_PCL_DB_PCL_DB_PIPELINE_H_

#include "mongodb_tf_transformer.h"

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <logging/logger.h>
#include <config/config.h>
#include <pcl_utils/utils.h>
#include <pcl_utils/transforms.h>
#include <pcl_utils/storage_adapter.h>
#include <pcl_utils/comparisons.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>

#define USE_ALIGNMENT
#define USE_ICP_ALIGNMENT
// define USE_NDT_ALIGNMENT

#define CFG_PREFIX "/perception/pcl-db/"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <mongo/client/dbclient.h>
#include <mongo/client/gridfs.h>

static const uint8_t cluster_colors[12][3] =
  { {176, 0, 30}, {0, 0, 255}, {255, 90, 0}, {137, 82, 39}, {56, 23, 90}, {99, 0, 30},
    {255, 0, 0}, {0, 255, 0}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {27, 117, 196}};

typedef enum {
  APPLICABLE = 0,
  TYPE_MISMATCH,
  NO_POINTCLOUD,
  QUERY_FAILED
} ApplicabilityStatus;

/** Database point cloud pipeline base class.
 * Common functionality for pcl-db-* plugins operating on
 * point clouds restored from MongoDB.
 * @author Tim Niemueller
 */
template <typename PointType>
class PointCloudDBPipeline
{
 protected:
  typedef pcl::PointCloud<PointType> Cloud;

  typedef pcl::PointXYZRGB ColorPointType;
  typedef pcl::PointCloud<ColorPointType> ColorCloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  typedef typename ColorCloud::Ptr ColorCloudPtr;
  typedef typename ColorCloud::ConstPtr ColorCloudConstPtr;

 public:
  /** Constructor.
   * @param mongodb_client MongoDB client
   * @param config configuration
   * @param logger Logger
   * @param output output point cloud
   */
 PointCloudDBPipeline(mongo::DBClientBase *mongodb_client,
		      fawkes::Configuration *config, fawkes::Logger *logger,
		      ColorCloudPtr output)
   : mongodb_client_(mongodb_client), logger_(logger), output_(output)
  {
    name_ = "PCL_DB_Pipeline";

    cfg_database_name_     = config->get_string(CFG_PREFIX"database-name");
    cfg_pcl_age_tolerance_ =
      (long)round(config->get_float(CFG_PREFIX"pcl-age-tolerance") * 1000.);
    std::vector<float> transform_range = config->get_floats(CFG_PREFIX"transform-range");
    if (transform_range.size() != 2) {
      throw fawkes::Exception("Transform range must be a list with exactly two elements");
    }
    if (transform_range[1] < transform_range[0]) {
      throw fawkes::Exception("Transform range start cannot be smaller than end");
    }
    cfg_transform_range_[0] = (long)round(transform_range[0] * 1000.);
    cfg_transform_range_[1] = (long)round(transform_range[1] * 1000.);

    mongodb_gridfs_ =
      new mongo::GridFS(*mongodb_client_, cfg_database_name_);
  }

  /** Destructor. */
  virtual ~PointCloudDBPipeline()
  {
    delete mongodb_gridfs_;
  }

  /** Check if this pipeline instance is suitable for the given times.
   * Retrieves information about the point clouds for the specified
   * \p times and checks if this pipeline (depending on the template
   * parameter) is suitable for the processing of these pipelines.
   * @param times times for which to check the point clouds
   * @param collection collection from which to retrieve the information
   * @return applicability status
   */
  ApplicabilityStatus
  applicable(std::vector<long long> &times, std::string &collection)
  {
    const unsigned int num_clouds = times.size();

#if PCL_VERSION_COMPARE(>=,1,7,0)
    std::vector<pcl::PCLPointField> pfields;
#else
    std::vector<sensor_msgs::PointField> pfields;
#endif
    pcl::for_each_type<typename pcl::traits::fieldList<PointType>::type>
      (pcl::detail::FieldAdder<PointType>(pfields));

    try {
      for (unsigned int i = 0; i < num_clouds; ++i) {
	std::auto_ptr<mongo::DBClientCursor> cursor =
	  mongodb_client_->query(cfg_database_name_ + "." + collection,
				 QUERY("timestamp" << mongo::LTE << times[i]
                           << mongo::GTE << (times[i] - cfg_pcl_age_tolerance_))
				 .sort("timestamp", -1),
				 /* limit */ 1);

	if (cursor->more()) {
	  mongo::BSONObj p = cursor->next();
	  mongo::BSONObj pcldoc = p.getObjectField("pointcloud");
	  std::vector<mongo::BSONElement> fields = pcldoc["field_info"].Array();

	  for (unsigned int i = 0; i < pfields.size(); ++i) {
#if PCL_VERSION_COMPARE(>=,1,7,0)
	    pcl::PCLPointField &pf = pfields[i];
#else
	    sensor_msgs::PointField &pf = pfields[i];
#endif

	    bool found = false;
	    for (unsigned int j = 0; j < fields.size(); ++j) {
	      if ((fields[j]["name"].String() == pf.name) &&
		  (fields[j]["offset"].Int() == (int)pf.offset) &&
		  (fields[j]["datatype"].Int() == pf.datatype) &&
		  (fields[j]["count"].Int() == (int)pf.count) )
	      {
		found = true;
		break;
	      }
	    }
	    if (! found) {
	      logger_->log_warn(name_, "Type mismatch for pointcloud "
				"at timestamp %lli", times[i]);
	      return TYPE_MISMATCH;
	    }
	  }
	} else {
	  logger_->log_warn(name_, "No pointclouds for timestamp %lli", times[i]);
	  return NO_POINTCLOUD;
	}
      }
    } catch (mongo::DBException &e) {
      logger_->log_warn(name_, "MongoDB query failed: %s", e.what());
      return QUERY_FAILED;
    }

    return APPLICABLE;
  }

 protected: // methods
  void
  read_gridfs_file(void *dataptr, std::string filename)
  {
    char *tmp = (char *)dataptr;
    mongo::GridFile file =
      mongodb_gridfs_->findFile(filename);
    if (! file.exists()) {
      logger_->log_warn(name_, "Grid file does not exist");
      return;
    }

    size_t bytes = 0;
    for (int c = 0; c < file.getNumChunks(); ++c) {
      mongo::GridFSChunk chunk = file.getChunk(c);
      int len = 0;
      const char *chunk_data = chunk.data(len);
      memcpy(tmp, chunk_data, len);
      tmp += len;
      bytes += len;
      //logger_->log_info(name_, "Read chunk %i of %i bytes", c, len);
    }
    //logger_->log_info(name_, "%zu bytes restored", bytes);
  }


  std::vector<CloudPtr>
  retrieve_clouds(std::vector<long long> &times, std::vector<long long> &actual_times,
		  std::string &collection)
  {
    mongodb_client_->ensureIndex(cfg_database_name_ + collection,
				 mongo::fromjson("{timestamp:1}"));

    const unsigned int num_clouds = times.size();
    std::vector<CloudPtr> pcls(num_clouds);

    // retrieve point clouds
    for (unsigned int i = 0; i < num_clouds; ++i) {

      std::auto_ptr<mongo::DBClientCursor> cursor =
	mongodb_client_->query(cfg_database_name_ + "." + collection,
			       QUERY("timestamp" << mongo::LTE << times[i]
                               << mongo::GTE << (times[i] - cfg_pcl_age_tolerance_))
			       .sort("timestamp", -1),
			       /* limit */ 1);

      if (cursor->more()) {
	mongo::BSONObj p = cursor->next();
	mongo::BSONObj pcldoc = p.getObjectField("pointcloud");
	std::vector<mongo::BSONElement> fields = pcldoc["field_info"].Array();

	long long timestamp = p["timestamp"].Long();
	double age = (double) (times[i] - timestamp) / 1000.;
	logger_->log_info(name_, "Restoring point cloud at %lli with age %f sec",
			  timestamp, age);

	// reconstruct point cloud
	CloudPtr lpcl(new Cloud());
	pcls[i] = lpcl;

	actual_times[i]       = p["timestamp"].Number();
	fawkes::Time actual_time((long)actual_times[i]);

	lpcl->header.frame_id = pcldoc["frame_id"].String();
	lpcl->is_dense        = (pcldoc["is_dense"].Int() != 0);
	lpcl->width           = pcldoc["width"].Int();
	lpcl->height          = pcldoc["height"].Int();
	fawkes::pcl_utils::set_time(lpcl, actual_time);
	lpcl->points.resize(pcldoc["num_points"].Int());

	read_gridfs_file(&lpcl->points[0], pcldoc.getFieldDotted("data.filename").String());
      } else {
	logger_->log_warn(name_, "Cannot retrieve document for time %li", times[i]);
	return std::vector<CloudPtr>();
      }
    }

    return pcls;
  }


 protected: // members
  const char *name_;

  std::string  cfg_database_name_;
  long         cfg_pcl_age_tolerance_;
  long         cfg_transform_range_[2];

  mongo::DBClientBase *mongodb_client_;
  mongo::GridFS *mongodb_gridfs_;

  fawkes::Logger        *logger_;

  ColorCloudPtr output_;

};

inline const char *
to_string(ApplicabilityStatus status)
{
  switch (status) {
  case APPLICABLE:    return "Applicable";
  case TYPE_MISMATCH: return "PointCloud in database does not match type";
  case NO_POINTCLOUD: return "For at least one time no pointcloud found";
  case QUERY_FAILED:  return "MongoDB query failed";
  default:            return "Unknown error";
  }
}

#endif
