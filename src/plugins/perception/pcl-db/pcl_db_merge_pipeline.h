
/***************************************************************************
 *  pcl_db_merge_pipeline.h - Restore and merge point clouds from MongoDB
 *                            Template class for variying point types
 *
 *  Created: Sat Dec 01 00:15:45 2012 (Freiburg)
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PERCEPTION_PCL_DB_PCL_DB_MERGE_PIPELINE_H_
#define __PLUGINS_PERCEPTION_PCL_DB_PCL_DB_MERGE_PIPELINE_H_

#include "mongodb_tf_transformer.h"
#include "pcl_db_pipeline.h"

#include <tf/transformer.h>
#include <pcl_utils/utils.h>
#include <pcl_utils/transforms.h>
#include <pcl_utils/comparisons.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>

#define USE_ALIGNMENT
#define USE_ICP_ALIGNMENT
// define USE_NDT_ALIGNMENT

#define CFG_PREFIX_MERGE "/perception/pcl-db-merge/"

// missing in Eigen3 causing a compiler error if not included here
#include <assert.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#ifdef USE_ICP_ALIGNMENT
#  include <pcl/registration/icp.h>
#endif
#ifdef USE_NDT_ALIGNMENT
#  if not defined(PCL_VERSION_COMPARE) || PCL_VERSION_COMPARE(<,1,7,0) 
#    error NDT alignment requires PCL 1.7.0 or higher
#  endif
#  include <pcl/registration/ndt.h>
#endif

#include <Eigen/StdVector>
#include <mongo/client/dbclient.h>

/** Point cloud merging pipeline.
 * This class can merge multiple point clouds which are restored from
 * a MongoDB database created by mongodb-log.
 * @author Tim Niemueller
 */
template <typename PointType>
class PointCloudDBMergePipeline : public PointCloudDBPipeline<PointType>
{
 public:
  /** Constructor.
   * @param mongodb_client MongoDB client
   * @param config configuration
   * @param logger Logger
   * @param transformer TF transformer for point cloud transformations between
   * coordinate reference frames
   * @param output output point cloud
   */
  PointCloudDBMergePipeline(mongo::DBClientBase *mongodb_client,
			    fawkes::Configuration *config, fawkes::Logger *logger,
			    fawkes::tf::Transformer *transformer,
			    typename PointCloudDBPipeline<PointType>::ColorCloudPtr output)
  : PointCloudDBPipeline<PointType>(mongodb_client, config, logger, output),
    tf_(transformer)
  {
    this->name_ = "PCL_DB_MergePL";

    cfg_transform_to_sensor_frame_ =
      config->get_bool(CFG_PREFIX_MERGE"transform-to-sensor-frame");
    if (cfg_transform_to_sensor_frame_) {
      cfg_fixed_frame_  = config->get_string(CFG_PREFIX_MERGE"fixed-frame");
      cfg_sensor_frame_ = config->get_string(CFG_PREFIX_MERGE"sensor-frame");
    }

    cfg_global_frame_ =
      config->get_string(CFG_PREFIX_MERGE"global-frame");
    cfg_passthrough_filter_axis_ =
      config->get_string(CFG_PREFIX_MERGE"passthrough-filter/axis");
    std::vector<float> passthrough_filter_limits =
      config->get_floats(CFG_PREFIX_MERGE"passthrough-filter/limits");
    if (passthrough_filter_limits.size() != 2) {
      throw fawkes::Exception("Pasthrough filter limits must be a list "
			      "with exactly two elements");
    }
    if (passthrough_filter_limits[1] < passthrough_filter_limits[0]) {
      throw fawkes::Exception("Passthrough filter limits start cannot be smaller than end");
    }
    cfg_passthrough_filter_limits_[0] = passthrough_filter_limits[0];
    cfg_passthrough_filter_limits_[1] = passthrough_filter_limits[1];
    cfg_downsample_leaf_size_ = config->get_float(CFG_PREFIX_MERGE"downsample-leaf-size");
    cfg_plane_rem_max_iter_ =
      config->get_float(CFG_PREFIX_MERGE"plane-removal/segmentation-max-iterations");
    cfg_plane_rem_dist_thresh_ =
      config->get_float(CFG_PREFIX_MERGE"plane-removal/segmentation-distance-threshold");
    cfg_icp_ransac_iterations_ =
      config->get_uint(CFG_PREFIX_MERGE"icp/ransac-iterations");
    cfg_icp_max_correspondance_distance_ =
      config->get_float(CFG_PREFIX_MERGE"icp/max-correspondance-distance");
    cfg_icp_max_iterations_ =
      config->get_uint(CFG_PREFIX_MERGE"icp/max-iterations");
    cfg_icp_transformation_eps_ =
      config->get_float(CFG_PREFIX_MERGE"icp/transformation-epsilon");
    cfg_icp_euclidean_fitness_eps_ =
      config->get_float(CFG_PREFIX_MERGE"icp/euclidean-fitness-epsilon");

    this->logger_->log_info(this->name_, "Age Tolerance: %li  "
			    "Limits: [%f, %f]  tf range: [%li, %li]",
			    this->cfg_pcl_age_tolerance_, cfg_passthrough_filter_limits_[0],
			    cfg_passthrough_filter_limits_[1], this->cfg_transform_range_[0],
			    this->cfg_transform_range_[1]);

    use_alignment_ = true;
#ifdef USE_TIMETRACKER
    tt_ = new fawkes::TimeTracker();
    tt_loopcount_ = 0;
    ttc_merge_              = tt_->add_class("Full Merge");
    ttc_retrieval_          = tt_->add_class("Retrieval");
    ttc_transform_global_   = tt_->add_class("Transform to Map");
    ttc_downsample_         = tt_->add_class("Downsampling");
    ttc_align_1_            = tt_->add_class("First ICP");
    ttc_transform_1_        = tt_->add_class("Apply 1st TF");
    ttc_remove_planes_      = tt_->add_class("Plane Removal");
    ttc_align_2_            = tt_->add_class("Second ICP");
    ttc_transform_final_    = tt_->add_class("Apply final TF");
    ttc_output_    = tt_->add_class("Output");
#endif
  }

  /** Destructor. */
  virtual ~PointCloudDBMergePipeline()
  {
#ifdef USE_TIMETRACKER
    delete tt_;
#endif
  }

  /** Merge point clouds.
   * @param times times for which to retrieve the point clouds.
   * @param database database to retrieve from
   * @param collection collection from which to retrieve the data
   */
  void
  merge(std::vector<long long> &times, std::string &database, std::string &collection)
  {
    TIMETRACK_START(ttc_merge_);
    const unsigned int num_clouds = times.size();

    std::vector<long long> actual_times(num_clouds);

    this->output_->points.clear();
    this->output_->height = 1;
    this->output_->width  = 0;
    this->output_->is_dense = false;

    std::vector<typename PointCloudDBPipeline<PointType>::CloudPtr> pcls(num_clouds);
    std::vector<typename PointCloudDBPipeline<PointType>::CloudPtr> non_transformed(num_clouds);
    std::vector<typename PointCloudDBPipeline<PointType>::CloudPtr> non_aligned(num_clouds);
    std::vector<typename PointCloudDBPipeline<PointType>::CloudPtr> non_aligned_downsampled(num_clouds);
    std::vector<typename PointCloudDBPipeline<PointType>::CloudPtr> aligned_downsampled(num_clouds);
    std::vector<typename PointCloudDBPipeline<PointType>::CloudPtr> aligned_downsampled_remplane(num_clouds);
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> >
      transforms(num_clouds-1);

    for (unsigned int i = 0; i < num_clouds; ++i) {
      non_transformed[i] = typename PointCloudDBPipeline<PointType>::CloudPtr(
        new typename PointCloudDBPipeline<PointType>::Cloud());
      non_aligned[i] = typename PointCloudDBPipeline<PointType>::CloudPtr(
        new typename PointCloudDBPipeline<PointType>::Cloud());
      non_aligned_downsampled[i] = typename PointCloudDBPipeline<PointType>::CloudPtr(
        new typename PointCloudDBPipeline<PointType>::Cloud());
      aligned_downsampled[i] = typename PointCloudDBPipeline<PointType>::CloudPtr(
        new typename PointCloudDBPipeline<PointType>::Cloud());
      aligned_downsampled_remplane[i] = typename PointCloudDBPipeline<PointType>::CloudPtr(
        new typename PointCloudDBPipeline<PointType>::Cloud());
    }

    TIMETRACK_START(ttc_retrieval_);

    pcls = PointCloudDBPipeline<PointType>::retrieve_clouds(times, actual_times, database, collection);
    if (pcls.empty()) {
      this->logger_->log_warn(this->name_, "No point clouds found for desired timestamps");
      TIMETRACK_ABORT(ttc_retrieval_);
      TIMETRACK_ABORT(ttc_merge_);
      return;
    }

    TIMETRACK_INTER(ttc_retrieval_, ttc_transform_global_);

    for (unsigned int i = 0; i < num_clouds; ++i) {
      // retrieve transforms
      fawkes::tf::MongoDBTransformer
	transformer(this->mongodb_client_, database);

      transformer.restore(/* start */  actual_times[i] + this->cfg_transform_range_[0],
			  /* end */    actual_times[i] + this->cfg_transform_range_[1]);
      this->logger_->log_debug(this->name_, "Restored transforms for %zu frames "
			       "for range (%lli..%lli)",
			       transformer.get_frame_caches().size(),
			       /* start */  actual_times[i] + this->cfg_transform_range_[0],
			       /* end */    actual_times[i] + this->cfg_transform_range_[1]);

      // transform point clouds to common frame
      try {
	fawkes::pcl_utils::transform_pointcloud(cfg_global_frame_, *pcls[i], transformer);
      } catch (fawkes::Exception &e) {
	this->logger_->log_warn(this->name_, "Failed to transform from %s to %s",
				pcls[i]->header.frame_id.c_str(),
				cfg_global_frame_.c_str());
	this->logger_->log_warn(this->name_, e);
      }
      *non_aligned[i] = *pcls[i];
    }

    // merge point clouds

    TIMETRACK_END(ttc_transform_global_);

    if (use_alignment_) {
      // align point clouds, use the first as target

      TIMETRACK_START(ttc_downsample_);

      // ### 1: ALIGN including table points

      // FILTER and DOWNSAMPLE

      pcl::PassThrough<PointType> pass;
      pass.setFilterFieldName(cfg_passthrough_filter_axis_.c_str());
      pass.setFilterLimits(cfg_passthrough_filter_limits_[0],
			   cfg_passthrough_filter_limits_[1]);

      //pcl::ApproximateVoxelGrid<PointType> downsample;
      pcl::VoxelGrid<PointType> downsample;
      downsample.setLeafSize(cfg_downsample_leaf_size_,
			     cfg_downsample_leaf_size_,
			     cfg_downsample_leaf_size_);

      typename PointCloudDBPipeline<PointType>::CloudPtr filtered_z(new typename PointCloudDBPipeline<PointType>::Cloud());

      for (unsigned int i = 0; i < num_clouds; ++i) {

	// downsample for efficient registration/Alignment
	pass.setInputCloud(pcls[i]);
	pass.filter(*filtered_z);

	downsample.setInputCloud(filtered_z);
	downsample.filter(*non_aligned_downsampled[i]);
	this->logger_->log_info(this->name_, "Filtered cloud %u contains %zu points",
				i, non_aligned_downsampled[i]->points.size ());
      }
      TIMETRACK_INTER(ttc_downsample_, ttc_align_1_);

      // ALIGN using ICP including table
      for (unsigned int i = 1; i < num_clouds; ++i) {
	this->logger_->log_info(this->name_, "Aligning cloud %u to %u", i, i-1);
	Eigen::Matrix4f transform;
	typename PointCloudDBPipeline<PointType>::CloudConstPtr source, target;

	source = non_aligned_downsampled[i];
	target = non_aligned_downsampled[i-1];

#  ifdef USE_ICP_ALIGNMENT
	align_icp(source, target, transform);

#  elif defined(USE_NDT_ALIGNMENT)
	align_ndt(source, target);
#  endif

        transforms[i-1] = transform;
      }

      TIMETRACK_INTER(ttc_align_1_, ttc_transform_1_);

      // ### 2: ALIGN excluding table points

      *aligned_downsampled[0] = *non_aligned_downsampled[0];
      for (unsigned int i = 1; i < num_clouds; ++i) {
	pcl::transformPointCloud(*non_aligned_downsampled[i],
				 *aligned_downsampled[i], transforms[i-1]);
      }

      TIMETRACK_INTER(ttc_transform_1_, ttc_remove_planes_);

      for (unsigned int i = 0; i < num_clouds; ++i) {
	*aligned_downsampled_remplane[i] = *aligned_downsampled[i];
	remove_plane(aligned_downsampled_remplane[i]);
	this->logger_->log_info(this->name_, "Removed plane from cloud %u, "
				"%zu of %zu points remain",
				i, aligned_downsampled_remplane[i]->points.size(),
				aligned_downsampled[i]->points.size());
      }

      TIMETRACK_INTER(ttc_remove_planes_, ttc_align_2_);

      for (unsigned int i = 1; i < num_clouds; ++i) {
	Eigen::Matrix4f transform;
	typename PointCloudDBPipeline<PointType>::CloudConstPtr source, target;

	source = aligned_downsampled_remplane[i];
	target = aligned_downsampled_remplane[i-1];
	
	align_icp(source, target, transform);

	typename PointCloudDBPipeline<PointType>::Cloud tmp;
	pcl::transformPointCloud(*aligned_downsampled_remplane[i], tmp, transform);
	*aligned_downsampled_remplane[i] = tmp;

	transforms[i-1] *= transform;
      }

      TIMETRACK_INTER(ttc_align_2_, ttc_transform_final_);

      for (unsigned int i = 1; i < num_clouds; ++i) {
	typename PointCloudDBPipeline<PointType>::Cloud tmp;
	pcl::transformPointCloud(*pcls[i], tmp, transforms[i-1]);
	*pcls[i] = tmp;
      }

      TIMETRACK_END(ttc_transform_final_);

    }

    TIMETRACK_END(ttc_merge_);
    TIMETRACK_START(ttc_output_);


#ifdef DEBUG_OUTPUT
    fawkes::Time now;

    merge_output(database, non_transformed, num_clouds);
    now.stamp(); fawkes::pcl_utils::set_time(this->output_, now);
    usleep(1000000);

    merge_output(database, non_aligned, num_clouds);
    now.stamp(); fawkes::pcl_utils::set_time(this->output_, now);
    usleep(1000000);

    merge_output(database, non_aligned_downsampled, num_clouds);
    now.stamp(); fawkes::pcl_utils::set_time(this->output_, now);
    usleep(1000000);

    merge_output(database, aligned_downsampled, num_clouds);
    now.stamp(); fawkes::pcl_utils::set_time(this->output_, now);
    usleep(1000000);

    merge_output(database, aligned_downsampled_remplane, num_clouds);
    now.stamp(); fawkes::pcl_utils::set_time(this->output_, now);
    usleep(1000000);
#endif

    merge_output(database, pcls, actual_times);

    TIMETRACK_END(ttc_output_);

#ifdef USE_TIMETRACKER
    if (++tt_loopcount_ >= 5) {
      tt_loopcount_ = 0;
      tt_->print_to_stdout();
    }
#endif
  }

 private: // methods
  void remove_plane(typename PointCloudDBPipeline<PointType>::CloudPtr &cloud)
  {
    pcl::SACSegmentation<PointType> tablesegm;
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    tablesegm.setOptimizeCoefficients(true);
    tablesegm.setModelType(pcl::SACMODEL_PLANE);
    tablesegm.setMethodType(pcl::SAC_RANSAC);
    tablesegm.setMaxIterations(1000);
    tablesegm.setDistanceThreshold(0.022);

    tablesegm.setInputCloud(cloud);
    tablesegm.segment(*inliers, *coeff);

    if (! coeff || coeff->values.empty()) {
      return;
    }

    pcl::ExtractIndices<PointType> extract;
    typename PointCloudDBPipeline<PointType>::Cloud extracted;
    extract.setNegative(true);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(extracted);
    *cloud = extracted;

    pcl::ConvexHull<PointType> convex_hull;
    convex_hull.setDimension(2);
    convex_hull.setInputCloud(cloud);
    typename PointCloudDBPipeline<PointType>::CloudPtr
      hull(new typename PointCloudDBPipeline<PointType>::Cloud());
    convex_hull.reconstruct(*hull);

    // Use only points above tables
    // Why coeff->values[3] < 0 ? ComparisonOps::GT : ComparisonOps::LT?
    // The model coefficients are in Hessian Normal Form, hence coeff[0..2] are
    // the normal vector. We need to distinguish the cases where the normal vector
    // points towards the origin (camera) or away from it. This can be checked
    // by calculating the distance towards the origin, which conveniently in
    // dist = N * x + p is just p which is coeff[3]. Therefore, if coeff[3] is
    // negative, the normal vector points towards the frame origin and we want all
    // points with positive distance from the table plane, otherwise it points
    // away from the origin and we want points with "negative distance".
    // We make use of the fact that we only have a boring RGB-D camera and
    // not an X-Ray...
    // Note that this assumes that the global frame's XY plane is the ground support
    // plane!
    pcl::ComparisonOps::CompareOp op =
      coeff->values[3] < 0 ? pcl::ComparisonOps::GT : pcl::ComparisonOps::LT;
    typename fawkes::pcl_utils::PlaneDistanceComparison<PointType>::ConstPtr
      above_comp(new fawkes::pcl_utils::PlaneDistanceComparison<PointType>(coeff, op));
    typename pcl::ConditionAnd<PointType>::Ptr
      above_cond(new pcl::ConditionAnd<PointType>());
    above_cond->addComparison(above_comp);
    pcl::ConditionalRemoval<PointType> above_condrem(above_cond);
    above_condrem.setInputCloud(cloud);
    typename PointCloudDBPipeline<PointType>::CloudPtr
      cloud_above(new typename PointCloudDBPipeline<PointType>::Cloud());
    above_condrem.filter(*cloud_above);

    // Extract only points on the table plane
    pcl::PointIndices::Ptr polygon(new pcl::PointIndices());

    typename pcl::ConditionAnd<PointType>::Ptr
      polygon_cond(new pcl::ConditionAnd<PointType>());

    typename fawkes::pcl_utils::PolygonComparison<PointType>::ConstPtr
      inpoly_comp(new fawkes::pcl_utils::PolygonComparison<PointType>(*hull));
    polygon_cond->addComparison(inpoly_comp);

    // build the filter
    pcl::ConditionalRemoval<PointType> condrem(polygon_cond);
    condrem.setInputCloud(cloud_above);
    condrem.filter(*cloud);
  }


#ifdef USE_ICP_ALIGNMENT
  bool align_icp(typename PointCloudDBPipeline<PointType>::CloudConstPtr source,
		 typename PointCloudDBPipeline<PointType>::CloudConstPtr target,
		 Eigen::Matrix4f &transform)
  {
    typename PointCloudDBPipeline<PointType>::Cloud final;

    //pcl::console::VERBOSITY_LEVEL old_level = pcl::console::getVerbosityLevel();
    //pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputCloud(source);
    icp.setInputTarget(target);

    icp.setRANSACIterations(cfg_icp_ransac_iterations_);

    // Set the max correspondence distance to 5cm
    // (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(cfg_icp_max_correspondance_distance_);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(cfg_icp_max_iterations_);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(cfg_icp_transformation_eps_);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(cfg_icp_euclidean_fitness_eps_);

    this->logger_->log_info(this->name_, "Aligning");
    icp.align(final);
    this->logger_->log_info(this->name_, "Aligning done");
    //this->logger_->log_info(this->name_, "ICP %u -> %u did%s converge, score: %f",
    //	       icp.hasConverged() ? "" : " NOT", icp.getFitnessScore());
    transform = icp.getFinalTransformation();
    //score     = icp.getFitnessScore();
    //pcl::console::setVerbosityLevel(old_level);
    return icp.hasConverged();
  }
#endif

#ifdef USE_NDT_ALIGNMENT
  // untested
  bool align_ndt(CloudConstPtr source, CloudConstPtr target, Eigen::Matrix4f &transform)
  {
    Cloud final;

    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setInputCloud(source);
    ndt.setInputTarget(target);

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(1.0);
      
    // Setting max number of registration iterations.
    ndt.setMaximumIterations(5);

    ndt.align(final);
    transform = ndt.getFinalTransformation();
    return ndt.hasConverged();
  }
#endif

  void
  merge_output(std::string &database,
	       std::vector<typename PointCloudDBPipeline<PointType>::CloudPtr> clouds,
	       std::vector<long long> &actual_times)
  {
    size_t num_points = 0;
    const size_t num_clouds = clouds.size();
    for (unsigned int i = 0; i < num_clouds; ++i) {
      num_points += clouds[i]->points.size();
    }
    this->output_->header.frame_id =
      cfg_transform_to_sensor_frame_ ? cfg_sensor_frame_ : cfg_global_frame_;
    this->output_->points.resize(num_points);
    this->output_->height = 1;
    this->output_->width = num_points;
    size_t out_p = 0;
    for (unsigned int i = 0; i < num_clouds; ++i) {
      const typename PointCloudDBPipeline<PointType>::CloudPtr &lpcl = clouds[i];
      const size_t cldn = lpcl->points.size();
      if (cldn == 0)  continue;

      for (size_t p = 0; p < cldn; ++p, ++out_p) {
	const PointType &ip      = lpcl->points[p];
	typename PointCloudDBPipeline<PointType>::ColorPointType &op =
	  this->output_->points[out_p];

	op.x = ip.x;
	op.y = ip.y;
	op.z = ip.z;

	op.r = cluster_colors[i][0];
	op.g = cluster_colors[i][1];
	op.b = cluster_colors[i][2];
      }
    }

    if (cfg_transform_to_sensor_frame_) {
      // retrieve transforms
      fawkes::tf::MongoDBTransformer
	transformer(this->mongodb_client_, database);

      unsigned int ref_pos = clouds.size() - 1;

      transformer.restore(/* start */  actual_times[ref_pos] + this->cfg_transform_range_[0],
			  /* end */    actual_times[ref_pos] + this->cfg_transform_range_[1]);
      this->logger_->log_debug(this->name_,
			       "Restored transforms for %zu frames for range (%lli..%lli)",
			       transformer.get_frame_caches().size(),
			       /* start */  actual_times[ref_pos] + this->cfg_transform_range_[0],
			       /* end */    actual_times[ref_pos] + this->cfg_transform_range_[1]);

      fawkes::Time source_time;
      fawkes::pcl_utils::get_time(clouds[ref_pos], source_time);
      fawkes::tf::StampedTransform transform_recorded;
      transformer.lookup_transform(cfg_fixed_frame_, cfg_global_frame_,
				   source_time, transform_recorded);

      fawkes::tf::StampedTransform transform_current;
      tf_->lookup_transform(cfg_sensor_frame_, cfg_fixed_frame_, transform_current);

      fawkes::tf::Transform transform = transform_current * transform_recorded;

      try {
	fawkes::pcl_utils::transform_pointcloud(*(this->output_), transform);
      } catch (fawkes::Exception &e) {
	this->logger_->log_warn(this->name_,
				"Failed to transform point cloud, exception follows");
	this->logger_->log_warn(this->name_, e);
      }
    }
  }


 private: // members

  fawkes::tf::Transformer *tf_;

  std::string  cfg_global_frame_;
  bool         cfg_transform_to_sensor_frame_;
  std::string  cfg_sensor_frame_;
  std::string  cfg_fixed_frame_;
  std::string  cfg_passthrough_filter_axis_;
  float        cfg_passthrough_filter_limits_[2];
  float        cfg_downsample_leaf_size_;
  float        cfg_plane_rem_max_iter_;
  float        cfg_plane_rem_dist_thresh_;
  unsigned int cfg_icp_ransac_iterations_;
  float        cfg_icp_max_correspondance_distance_;
  unsigned int cfg_icp_max_iterations_;
  float        cfg_icp_transformation_eps_;
  float        cfg_icp_euclidean_fitness_eps_;

#ifdef USE_TIMETRACKER
  fawkes::TimeTracker *tt_;
  unsigned int tt_loopcount_;
  unsigned int ttc_merge_;
  unsigned int ttc_retrieval_;
  unsigned int ttc_transform_global_;
  unsigned int ttc_downsample_;
  unsigned int ttc_align_1_;
  unsigned int ttc_transform_1_;
  unsigned int ttc_remove_planes_;
  unsigned int ttc_align_2_;
  unsigned int ttc_transform_final_;
  unsigned int ttc_output_;
#endif

  bool use_alignment_;
};

#endif
