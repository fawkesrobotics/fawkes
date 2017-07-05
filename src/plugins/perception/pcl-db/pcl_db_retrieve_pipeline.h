
/***************************************************************************
 *  pcl_db_retrieve_pipeline.h - Retrieve point clouds from MongoDB
 *                               Template class for variying point types
 *
 *  Created: Thu Aug 22 11:02:59 2013
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

#ifndef __PLUGINS_PERCEPTION_PCL_DB_PCL_DB_RETRIEVE_PIPELINE_H_
#define __PLUGINS_PERCEPTION_PCL_DB_PCL_DB_RETRIEVE_PIPELINE_H_

#include "mongodb_tf_transformer.h"
#include "pcl_db_pipeline.h"

#include <tf/transformer.h>
#include <pcl_utils/utils.h>
#include <pcl_utils/transforms.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>

#define CFG_PREFIX_RETRV "/perception/pcl-db-retrieve/"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mongo/client/dbclient.h>
#include <mongo/client/gridfs.h>

/** Point cloud retrieve pipeline.
 * This pipeline retrieves a point cloud from the database at a given point
 * in time. The process assums a coordinate frame which is fixed in both,
 * the recorded transforms and the current transform tree. Using this fixed
 * frame it first transforms the point cloud to the fixed frame using the
 * recorded transforms in the database, and then to transform it to the
 * desired sensor frame using the current data.
 * @author Tim Niemueller
 */
template <typename PointType>
class PointCloudDBRetrievePipeline : public PointCloudDBPipeline<PointType>
{
 public:
  /** Constructor.
   * @param mongodb_client MongoDB client
   * @param config configuration
   * @param logger Logger
   * @param output output point cloud
   * @param transformer TF transformer for point cloud transformations between
   * coordinate reference frames
   * @param original input point cloud
   * @param output output point cloud
   */
  PointCloudDBRetrievePipeline(mongo::DBClientBase *mongodb_client,
			       fawkes::Configuration *config, fawkes::Logger *logger,
			       fawkes::tf::Transformer *transformer,
			       typename PointCloudDBPipeline<PointType>::ColorCloudPtr original,
			       typename PointCloudDBPipeline<PointType>::ColorCloudPtr output)
  : PointCloudDBPipeline<PointType>(mongodb_client, config, logger, output),
    tf_(transformer), original_(original)
  {
    this->name_ = "PCL_DB_RetrievePL";

    cfg_fixed_frame_  = config->get_string(CFG_PREFIX_RETRV"fixed-frame");
    cfg_sensor_frame_ = config->get_string(CFG_PREFIX_RETRV"sensor-frame");

#ifdef USE_TIMETRACKER
    tt_ = new fawkes::TimeTracker();
    tt_loopcount_ = 0;
    ttc_retrieve_           = tt_->add_class("Full Retrieve");
    ttc_retrieval_          = tt_->add_class("Retrieval");
    ttc_transforms_         = tt_->add_class("Transforms");
#endif
  }

  /** Destructor. */
  virtual ~PointCloudDBRetrievePipeline()
  {
#ifdef USE_TIMETRACKER
    delete tt_;
#endif
  }

  /** Retrieve point clouds.
   * @param timestamp time for which to retrieve the point cloud
   * @param database database to retrieve from
   * @param collection collection from which to retrieve the data
   * @param target_frame coordinate frame to transform to
   * @param actual_time upon return contains the actual time for
   * which a point cloud was retrieved
   */
  void
  retrieve(long long timestamp, std::string &database,
	   std::string &collection, std::string &target_frame,
	   long long &actual_time)
  {
    TIMETRACK_START(ttc_retrieve_);

    this->output_->points.clear();
    this->output_->height = 1;
    this->output_->width  = 0;
    this->output_->is_dense = false;

    std::vector<long long> times(1, timestamp);
    std::vector<long long> actual_times(1, 0);
    std::vector<typename PointCloudDBPipeline<PointType>::CloudPtr> pcls(1);

    TIMETRACK_START(ttc_retrieval_);

    pcls = PointCloudDBPipeline<PointType>::retrieve_clouds(times, actual_times, database, collection);
    if (pcls.empty()) {
      this->logger_->log_warn(this->name_, "No point clouds found for desired timestamp");
      TIMETRACK_ABORT(ttc_retrieval_);
      TIMETRACK_ABORT(ttc_retrieve_);
      return;
    }

    copy_output(pcls[0], original_, 128, 128, 128);
    actual_time = actual_times[0];

    if (target_frame == "SENSOR") {
      target_frame == cfg_sensor_frame_;
    }

    if (target_frame != "") {
      // perform transformation
 
      TIMETRACK_INTER(ttc_retrieval_, ttc_transforms_);

      // retrieve transforms
      fawkes::tf::MongoDBTransformer
	transformer(this->mongodb_client_, database);

      transformer.restore(/* start */  actual_times[0] + this->cfg_transform_range_[0],
			  /* end */    actual_times[0] + this->cfg_transform_range_[1]);
      this->logger_->log_debug(this->name_,
			       "Restored transforms for %zu frames for range (%lli..%lli)",
			       transformer.get_frame_caches().size(),
			       /* start */  actual_times[0] + this->cfg_transform_range_[0],
			       /* end */    actual_times[0] + this->cfg_transform_range_[1]);

      fawkes::Time source_time;
      fawkes::pcl_utils::get_time(pcls[0], source_time);
      fawkes::tf::StampedTransform transform_recorded;
      transformer.lookup_transform(cfg_fixed_frame_, pcls[0]->header.frame_id,
				   source_time, transform_recorded);

      fawkes::tf::StampedTransform transform_current;
      tf_->lookup_transform(cfg_sensor_frame_, cfg_fixed_frame_, transform_current);

      fawkes::tf::Transform transform = transform_current * transform_recorded;

      try {
	fawkes::pcl_utils::transform_pointcloud(*pcls[0], transform);
      } catch (fawkes::Exception &e) {
	this->logger_->log_warn(this->name_,
				"Failed to transform point cloud, exception follows");
	this->logger_->log_warn(this->name_, e);
      }

      // retrieve point clouds
      TIMETRACK_END(ttc_transforms_);
    }

    copy_output(pcls[0], this->output_);

    TIMETRACK_END(ttc_retrieve_);

#ifdef USE_TIMETRACKER
    //if (++tt_loopcount_ >= 5) {
    //  tt_loopcount_ = 0;
      tt_->print_to_stdout();
      //}
#endif
  }

 private: //methods
   void copy_output(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in,
		    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out)
   {
     size_t num_points = in->points.size();
     out->header.frame_id = in->header.frame_id;
     out->points.resize(num_points);
     out->height = 1;
     out->width = num_points;

     for (size_t p = 0; p < num_points; ++p) {
       const PointType &ip = in->points[p];
       typename PointCloudDBPipeline<PointType>::ColorPointType &op = out->points[p];

       op.x = ip.x;
       op.y = ip.y;
       op.z = ip.z;

       op.r = ip.r;
       op.g = ip.g;
       op.b = ip.b;
     }
   }

   void copy_output(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in,
		    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out,
		    const int r, const int g, const int b)
   {
     size_t num_points = in->points.size();
     out->header.frame_id = in->header.frame_id;
     out->points.resize(num_points);
     out->height = 1;
     out->width = num_points;

     for (size_t p = 0; p < num_points; ++p) {
       const PointType &ip = in->points[p];
       typename PointCloudDBPipeline<PointType>::ColorPointType &op = out->points[p];

       op.x = ip.x;
       op.y = ip.y;
       op.z = ip.z;

       op.r = r;
       op.g = g;
       op.b = b;
     }
   }

   void copy_output(pcl::PointCloud<pcl::PointXYZ>::Ptr &in,
		    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out,
		    const int r = 255, const int g = 255, const int b = 255)
   {
     size_t num_points = in->points.size();
     out->header.frame_id = in->header.frame_id;
     out->points.resize(num_points);
     out->height = 1;
     out->width = num_points;

     for (size_t p = 0; p < num_points; ++p) {
       const PointType &ip = in->points[p];
       typename PointCloudDBPipeline<PointType>::ColorPointType &op = out->points[p];

       op.x = ip.x;
       op.y = ip.y;
       op.z = ip.z;

       op.r = r;
       op.g = g;
       op.b = b;
     }
   }


 private: // members
  std::string  cfg_fixed_frame_;
  std::string  cfg_sensor_frame_;

  fawkes::tf::Transformer *tf_;

  typename PointCloudDBPipeline<PointType>::ColorCloudPtr original_;

#ifdef USE_TIMETRACKER
  fawkes::TimeTracker *tt_;
  unsigned int tt_loopcount_;
  unsigned int ttc_retrieve_;
  unsigned int ttc_retrieval_;
  unsigned int ttc_transforms_;
#endif
};

#endif
