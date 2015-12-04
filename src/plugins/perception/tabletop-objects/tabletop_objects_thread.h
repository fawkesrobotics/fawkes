
/***************************************************************************
 *  tabletop_objects_thread.h - Thread to detect tabletop objects
 *
 *  Created: Fri Nov 04 23:54:19 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PERCEPTION_TABLETOP_OBJECTS_THREAD_H_
#define __PLUGINS_PERCEPTION_TABLETOP_OBJECTS_THREAD_H_

// must be first for reliable ROS detection
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/tf.h>
#include <aspect/pointcloud.h>

#include <Eigen/StdVector>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <map>
#include <list>

namespace fawkes {
  class Position3DInterface;
  class SwitchInterface;
  class Time;
#ifdef USE_TIMETRACKER
  class TimeTracker;
#endif
}

#ifdef HAVE_VISUAL_DEBUGGING
class TabletopVisualizationThreadBase;
#endif

/** @class OldCentroid "tabletop_objects_thread.h"
 * This class is used to save old centroids in order to check for reappearance
 */
class OldCentroid {
public:
  /**
   * Constructor
   * @param id The ID which the centroid was assigned to
   * @param centroid The position of the centroid
   */
  OldCentroid(const unsigned int &id, const Eigen::Vector4f &centroid)
  : id_(id), age_(0), centroid_(centroid) { }
  /** Copy constructor
   * @param other The other OldCentroid */
  OldCentroid(const OldCentroid &other)
  : id_(other.getId()), age_(other.getAge()), centroid_(other.getCentroid()) { }
  virtual ~OldCentroid() { }
  // any class with Eigen::Vector4f needs a custom new operator
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** get the ID of the centroid
   * @return the ID of the centroid */
  unsigned int getId() const { return id_; }
  /** get the position of the centroid
   * @return a reference to the centroid */
  const Eigen::Vector4f& getCentroid() const { return centroid_; }
  /** get the age of the centroid
   * @return the number of loops the centroids has been invisible
   */
  unsigned int getAge() const { return age_; }
  /** increment the age of the centroid */
  void age() { age_++; }

protected:
  /** The ID of the centroid */
  unsigned int id_;
  /** The number of loops the centroid has been invisible for */
  unsigned int age_;
  /** The position of centroid */
  Eigen::Vector4f centroid_;
};

class TabletopObjectsThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::TransformAspect,
  public fawkes::PointCloudAspect
{
 public:
  TabletopObjectsThread();
  virtual ~TabletopObjectsThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

#ifdef HAVE_VISUAL_DEBUGGING
  void set_visualization_thread(TabletopVisualizationThreadBase *visthread);
#endif

 private:
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> Cloud;

  typedef pcl::PointXYZRGB ColorPointType;
  typedef pcl::PointCloud<ColorPointType> ColorCloud;
  typedef Cloud::Ptr CloudPtr;
  typedef Cloud::ConstPtr CloudConstPtr;

  typedef ColorCloud::Ptr ColorCloudPtr;
  typedef ColorCloud::ConstPtr ColorCloudConstPtr;

  typedef std::map<unsigned int, Eigen::Vector4f, std::less<unsigned int>,
      Eigen::aligned_allocator<std::pair<const unsigned int, Eigen::Vector4f>>>
      CentroidMap;
  typedef std::list<OldCentroid, Eigen::aligned_allocator<OldCentroid> > OldCentroidVector;
  typedef std::vector<fawkes::Position3DInterface *> PosIfsVector;

 private:
  void set_position(fawkes::Position3DInterface *iface,
                    bool is_visible, const Eigen::Vector4f &centroid = Eigen::Vector4f(0, 0, 0, 0),
                    const Eigen::Quaternionf &rotation = Eigen::Quaternionf(1, 0, 0, 0));

  CloudPtr simplify_polygon(CloudPtr polygon, float sqr_dist_threshold);
  CloudPtr generate_table_model(const float length, const float width,
                                const float thickness, const float step, const float max_error);
  CloudPtr generate_table_model(const float length, const float width,
                                const float step, const float max_error = 0.01);
  bool is_polygon_edge_better(PointType &cb_br_p1p, PointType &cb_br_p2p, PointType &br_p1p, PointType &br_p2p);
  bool compute_bounding_box_scores(Eigen::Vector3f& cluster_dim, std::vector < Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >& scores);
  double compute_similarity(double d1, double d2);

  void convert_colored_input();

  std::vector<pcl::PointIndices> extract_object_clusters(CloudConstPtr input);

  ColorCloudPtr colorize_cluster(CloudConstPtr input_cloud, const std::vector<int> &cluster, const uint8_t color[]);

  unsigned int cluster_objects(CloudConstPtr input, ColorCloudPtr tmp_clusters, std::vector<ColorCloudPtr> &tmp_obj_clusters);

  int next_id();
  void delete_old_centroids(OldCentroidVector centroids, unsigned int age);
  void delete_near_centroids(CentroidMap reference, OldCentroidVector centroids,
    float min_distance);
  void remove_high_centroids(Eigen::Vector4f table_centroid, CentroidMap centroids);
  Eigen::Vector4f fit_cylinder(ColorCloudConstPtr obj_in_base_frame, Eigen::Vector4f const &centroid, uint const &centroid_i);
  std::map<unsigned int, int> track_objects(std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> new_centroids);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::RefPtr<const pcl::PointCloud<PointType> > finput_;
  fawkes::RefPtr<const pcl::PointCloud<ColorPointType> > fcoloredinput_;
  fawkes::RefPtr<pcl::PointCloud<ColorPointType> > fclusters_;
  ColorCloudConstPtr colored_input_;
  CloudPtr           converted_input_;
  CloudConstPtr      input_;
  pcl::PointCloud<ColorPointType>::Ptr clusters_;

  std::vector<fawkes::RefPtr<pcl::PointCloud<ColorPointType> > > f_obj_clusters_;
  std::vector<pcl::PointCloud<ColorPointType>::Ptr> obj_clusters_;
  std::map<unsigned int, double> obj_shape_confidence_;

//  std::map<unsigned int, std::vector<double>> obj_likelihoods_;
  std::map<unsigned int, signed int> best_obj_guess_;
  int NUM_KNOWN_OBJS_;

  double table_inclination_;

  std::vector < Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > known_obj_dimensions_;

  pcl::VoxelGrid<PointType> grid_;
  pcl::SACSegmentation<PointType> seg_;

  PosIfsVector pos_ifs_;
  fawkes::Position3DInterface *table_pos_if_;

  Eigen::Vector4f table_centroid;

  std::list<unsigned int> free_ids_;

  fawkes::SwitchInterface *switch_if_;

  fawkes::Time *last_pcl_time_;

  float cfg_depth_filter_min_x_;
  float cfg_depth_filter_max_x_;
  float cfg_voxel_leaf_size_;
  unsigned int cfg_segm_max_iterations_;
  float cfg_segm_distance_threshold_;
  float cfg_segm_inlier_quota_;
  float cfg_max_z_angle_deviation_;
  float cfg_table_min_cluster_quota_;
  float cfg_table_downsample_leaf_size_;
  float cfg_table_cluster_tolerance_;
  float cfg_table_min_height_;
  float cfg_table_max_height_;
  bool  cfg_table_model_enable_;
  float cfg_table_model_length_;
  float cfg_table_model_width_;
  float cfg_table_model_step_;
  float cfg_horizontal_va_;
  float cfg_vertical_va_;
  float cfg_cluster_tolerance_;
  unsigned int cfg_cluster_min_size_;
  unsigned int cfg_cluster_max_size_;
  std::string cfg_base_frame_;
  std::string cfg_result_frame_;
  std::string cfg_input_pointcloud_;
  uint cfg_centroid_max_age_;
  float cfg_centroid_max_distance_;
  float cfg_centroid_min_distance_;
  float cfg_centroid_max_height_;
  bool cfg_cylinder_fitting_;
  bool cfg_track_objects_;
  bool cfg_verbose_cylinder_fitting_;

  fawkes::RefPtr<Cloud> ftable_model_;
  CloudPtr table_model_;
  fawkes::RefPtr<Cloud> fsimplified_polygon_;
  CloudPtr simplified_polygon_;

  unsigned int loop_count_;

  CentroidMap centroids_;
  CentroidMap cylinder_params_;
  OldCentroidVector old_centroids_;
  bool first_run_;

  std::map<uint, std::vector<double>> obj_likelihoods_;

#ifdef USE_TIMETRACKER
  fawkes::TimeTracker  *tt_;
  unsigned int tt_loopcount_;
  unsigned int ttc_full_loop_;
  unsigned int ttc_msgproc_;
  unsigned int ttc_convert_;
  unsigned int ttc_voxelize_;
  unsigned int ttc_plane_;
  unsigned int ttc_extract_plane_;
  unsigned int ttc_plane_downsampling_;
  unsigned int ttc_cluster_plane_;
  unsigned int ttc_convex_hull_;
  unsigned int ttc_simplify_polygon_;
  unsigned int ttc_find_edge_;
  unsigned int ttc_transform_;
  unsigned int ttc_transform_model_;
  unsigned int ttc_extract_non_plane_;
  unsigned int ttc_polygon_filter_;
  unsigned int ttc_table_to_output_;
  unsigned int ttc_cluster_objects_;
  unsigned int ttc_visualization_;
  unsigned int ttc_hungarian_;
  unsigned int ttc_old_centroids_;
  unsigned int ttc_obj_extraction_;
#endif

#ifdef HAVE_VISUAL_DEBUGGING
  TabletopVisualizationThreadBase *visthread_;
#endif
};


#endif
