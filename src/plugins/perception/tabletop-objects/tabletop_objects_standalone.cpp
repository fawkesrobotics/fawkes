
/***************************************************************************
 *  tabletop_objects_standalone.cpp - Thread to detect tabletop objects
 *
 *  Created: Sat Oct 01 11:51:00 2011
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

/// @cond EXAMPLE

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <fvcams/shmem.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/adapters/pcl.h>
#include <utils/time/time.h>

#include <limits>

#define TABLE_MAX_X  3.0
#define TABLE_MAX_Y  3.0
#define TABLE_MIN_X -3.0
#define TABLE_MIN_Y -3.0

using namespace fawkes;
using namespace firevision;

template <typename PointT>
class PolygonComparison : public pcl::ComparisonBase<PointT>
{
  using pcl::ComparisonBase<PointT>::capable_;
 public:
  typedef boost::shared_ptr<PolygonComparison<PointT> > Ptr;
  typedef boost::shared_ptr<const PolygonComparison<PointT> > ConstPtr;
 
  PolygonComparison(const pcl::PointCloud<PointT> &polygon, bool inside = true)
    : inside_(inside), polygon_(polygon)
  {
    capable_ = (polygon.size() >= 3);
  }
  virtual ~PolygonComparison() {}
 
  virtual bool evaluate(const PointT &point) const
  {
    if (inside_)
      return   pcl::isPointIn2DPolygon(point, polygon_);
    else
      return ! pcl::isPointIn2DPolygon(point, polygon_);
  }
 
 protected:
  bool inside_;
  const pcl::PointCloud<PointT> &polygon_;
 
 private:
  PolygonComparison() {} // not allowed
};


template <typename PointT>
class PlaneDistanceComparison : public pcl::ComparisonBase<PointT>
{
  using pcl::ComparisonBase<PointT>::capable_;
 public:
  typedef boost::shared_ptr<PlaneDistanceComparison<PointT> > Ptr;
  typedef boost::shared_ptr<const PlaneDistanceComparison<PointT> > ConstPtr;
 
  PlaneDistanceComparison(pcl::ModelCoefficients::ConstPtr coeff,
                          pcl::ComparisonOps::CompareOp op = pcl::ComparisonOps::GT,
                          float compare_val = 0.)
    : coeff_(coeff), op_(op), compare_val_(compare_val)
  {
    capable_ = (coeff_->values.size() == 4);
  }
  virtual ~PlaneDistanceComparison() {}
 
  virtual bool evaluate(const PointT &point) const
  {
    float val = (coeff_->values[0] * point.x + coeff_->values[1] * point.y +
                 coeff_->values[2] * point.z + coeff_->values[3]) /
      sqrtf(coeff_->values[0] * coeff_->values[0] +
            coeff_->values[1] * coeff_->values[1] +
            coeff_->values[2] * coeff_->values[2]);

    //printf("%f > %f?: %d\n", val, compare_val_, (val > compare_val_));

    if (op_ == pcl::ComparisonOps::GT) {
      return val >  compare_val_;
    } else if (op_ == pcl::ComparisonOps::GE) {
      return val >= compare_val_;
    } else if (op_ == pcl::ComparisonOps::LT) {
      return val <  compare_val_;
    } else if (op_ == pcl::ComparisonOps::LE) {
      return val <= compare_val_;
    } else {
      return val == compare_val_;
    }
  }
 
 protected:
  pcl::ModelCoefficients::ConstPtr coeff_;
  pcl::ComparisonOps::CompareOp op_;
  float compare_val_;

 private:
  PlaneDistanceComparison() {} // not allowed
};

template <typename PointType>
class OpenNIPlanarSegmentation
{
public:
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  OpenNIPlanarSegmentation (const std::string& device_id = "", double threshold = 0.01)
    : viewer ("PCL OpenNI Planar Segmentation Viewer"),
      device_id_ (device_id),
      new_cloud_(false)
  {
    grid_.setFilterFieldName ("z");
    grid_.setFilterLimits (0.0, 3.0);
    grid_.setLeafSize (0.01, 0.01, 0.01);

    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_PLANE);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setMaxIterations (1000);
    seg_.setDistanceThreshold (threshold);

  }

  void 
  cloud_cb_ (const CloudConstPtr& cloud)
  {
    set (cloud);
  }

  void
  set (const CloudConstPtr& cloud)
  {
    //lock while we set our cloud;
    boost::mutex::scoped_lock lock (mtx_);
    cloud_  = cloud;
  }

  int get_nn(float x, float y, float z) const
  {
    PointType p(x, y, z);
    std::vector<int> indices;
    std::vector<float> distances;

    float min_dist = std::numeric_limits<float>::max();
    int count = kdtree_.radiusSearch(p, 1.0, indices, distances);
    if (! indices.empty()) {
      printf("Got %i indices!\n", count);
      int index = 0;
      for (int i = 0; i < count; ++i) {
        if (distances[i] < min_dist) {
          index = i;
          min_dist = distances[i];
        }
      }
      printf("Found at dist %f (%f, %f, %f)\n", distances[index],
             cloud_proj_->points[indices[index]].x, 
             cloud_proj_->points[indices[index]].y, 
             cloud_proj_->points[indices[index]].z);
      return indices[index];
    } else {
      printf("No index found looking for (%f, %f, %f)\n", x, y, z);
    }

    return -1;
  }

  CloudPtr
  get ()
  {
    //lock while we swap our cloud and reset it.
    boost::mutex::scoped_lock lock (mtx_);
    CloudPtr temp_cloud (new Cloud);
    CloudPtr temp_cloud2 (new Cloud);

    grid_.setInputCloud (cloud_);
    grid_.filter (*temp_cloud);

    // set all colors to white for better distinguishing the pixels
    typename pcl::PointCloud<PointType>::iterator p;
    for (p = temp_cloud->begin(); p != temp_cloud->end(); ++p) {
      p->r = 255;
      p->g = 255;
      p->b = 255;
    }

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    seg_.setInputCloud (temp_cloud);
    seg_.segment (*inliers, *coefficients);

    extract_.setNegative (false);
    extract_.setInputCloud (temp_cloud);
    extract_.setIndices (inliers);
    extract_.filter (*temp_cloud2);

    // Project the model inliers
    pcl::ProjectInliers<PointType> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(temp_cloud2);
    proj.setModelCoefficients(coefficients);
    cloud_proj_.reset(new Cloud());
    proj.filter (*cloud_proj_);
    //printf("PointCloud after projection has: %zu data points.\n",
    //       cloud_proj_->points.size());


    // Estimate 3D convex hull -> TABLE BOUNDARIES
    pcl::ConvexHull<PointType> hr;
    //hr.setAlpha (0.1);  // only for ConcaveHull
    hr.setInputCloud(cloud_proj_);
    cloud_hull_.reset(new Cloud());
    hr.reconstruct (*cloud_hull_, vertices_);

    //printf("Found %zu vertices, first has size %zu\n",
    //       vertices_.size(), vertices_[0].vertices.size());

    for (size_t i = 0; i < cloud_proj_->points.size(); ++i) {
      cloud_proj_->points[i].r =   0;
      cloud_proj_->points[i].g = 255;
      cloud_proj_->points[i].b =   0;
    }

    // Extrat all non-plane points
    cloud_filt_.reset(new Cloud());
    extract_.setNegative(true);
    extract_.filter(*cloud_filt_);

    // remove all pixels below table
    typename PlaneDistanceComparison<PointType>::ConstPtr
      above_comp(new PlaneDistanceComparison<PointType>(coefficients, pcl::ComparisonOps::LT));
    typename pcl::ConditionAnd<PointType>::Ptr
      above_cond(new pcl::ConditionAnd<PointType>());
    above_cond->addComparison(above_comp);
    pcl::ConditionalRemoval<PointType> above_condrem(above_cond);
    above_condrem.setInputCloud(cloud_filt_);
    //above_condrem.setKeepOrganized(true);
    cloud_above_.reset(new Cloud());
    above_condrem.filter(*cloud_above_);

    printf("Before: %zu  After: %zu\n", cloud_filt_->points.size(),
           cloud_above_->points.size());

      // Extract only points on the table plane
    if (! vertices_.empty()) {
      pcl::PointIndices::Ptr polygon(new pcl::PointIndices());
      polygon->indices = vertices_[0].vertices;

      pcl::PointCloud<PointType> polygon_cloud;
      pcl::ExtractIndices<PointType> polygon_extract;

      polygon_extract.setInputCloud(cloud_hull_);
      polygon_extract.setIndices(polygon);
      polygon_extract.filter(polygon_cloud);

      typename pcl::ConditionAnd<PointType>::Ptr
        polygon_cond(new pcl::ConditionAnd<PointType>());

      typename PolygonComparison<PointType>::ConstPtr
        inpoly_comp(new PolygonComparison<PointType>(polygon_cloud));
      polygon_cond->addComparison(inpoly_comp);

      // build the filter
      pcl::ConditionalRemoval<PointType> condrem(polygon_cond);
      condrem.setInputCloud(cloud_above_);
      //condrem.setKeepOrganized(true);
      cloud_objs_.reset(new Cloud());
      condrem.filter(*cloud_objs_);
    } else {
      cloud_objs_.reset(new Cloud(*cloud_above_));
    }

    // CLUSTERS
    // extract clusters of OBJECTS

    // Creating the KdTree object for the search method of the extraction
    pcl::KdTree<pcl::PointXYZRGB>::Ptr
      kdtree_cl(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());
    kdtree_cl->setInputCloud(cloud_objs_);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.04); // 2cm
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(kdtree_cl);
    ec.setInputCloud(cloud_objs_);
    ec.extract(cluster_indices);

    printf("Found %zu clusters\n", cluster_indices.size());

    uint8_t colors[5][3] = { {255, 0, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255},
                             {0, 255, 255} };

    std::vector<pcl::PointIndices>::const_iterator it;
    unsigned int color = 0;
    unsigned int i = 0;
    for (it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
      uint8_t r, g, b;
      if (color < 5) {
        r = colors[color][0];
        g = colors[color][1];
        b = colors[color][2];
        ++color;
      } else {
        double dr=0, dg=0, db=0;
        pcl::visualization::getRandomColors(dr, dg, db);
        r = (uint8_t)roundf(dr * 255);
        g = (uint8_t)roundf(dg * 255);
        b = (uint8_t)roundf(db * 255);
      }
      printf("Cluster %u  size: %zu  color %u, %u, %u\n",
             ++i, it->indices.size(), r, g, b);
      std::vector<int>::const_iterator pit;
      for (pit = it->indices.begin (); pit != it->indices.end(); pit++) {
        cloud_objs_->points[*pit].r = r;
        cloud_objs_->points[*pit].g = g;
        cloud_objs_->points[*pit].b = b;
      }
    }

    /*  To project into Z:
    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr proj_coeff (new pcl::ModelCoefficients ());
    proj_coeff->values.resize (4);
    proj_coeff->values[0] = proj_coeff->values[1] = 0;
    proj_coeff->values[2] = 1.0;
    proj_coeff->values[3] = 0;

    // Create the filtering object
    pcl::ProjectInliers<PointType> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_hull_);
    proj.setModelCoefficients(proj_coeff);
    cloud_proj_.reset(new Cloud());
    proj.filter(*cloud_proj_);

    printf("Vertices: %zu, first: %zu\n", vertices_.size(), vertices_[0].vertices.size());
    vertices_.resize(5);

    //get the extents of the table
     if (! cloud_proj_->points.empty()) {
      printf("Cloud hull non-empty\n");
      float x_min = std::numeric_limits<float>::max(), y_min = std::numeric_limits<float>::max(), x_max = 0, y_max = 0;

      for (size_t i = 1; i < cloud_proj_->points.size(); ++i) {
        if (cloud_proj_->points[i].x < x_min && cloud_proj_->points[i].x > -3.0)
          x_min = cloud_proj_->points[i].x;
        if (cloud_proj_->points[i].x > x_max && cloud_proj_->points[i].x < 3.0)
          x_max = cloud_proj_->points[i].x;
        if (cloud_proj_->points[i].y < y_min && cloud_proj_->points[i].y > -3.0)
          y_min = cloud_proj_->points[i].y;
        if (cloud_proj_->points[i].y > y_max && cloud_proj_->points[i].y < 3.0)
          y_max = cloud_proj_->points[i].y;
      }

      kdtree_.setInputCloud(cloud_proj_);
      float table_z = cloud_proj_->points[0].z; // only need a very rough estimate

      printf("x_min=%f  x_max=%f  y_min=%f  y_max=%f  table_z = %f\n",
             x_min, x_max, y_min, y_max, table_z);

      int blp = get_nn(x_min, y_min, table_z);
      int brp = get_nn(x_max, y_min, table_z);
      int tlp = get_nn(x_min, y_max, table_z);
      int trp = get_nn(x_max, y_max, table_z);

      printf("blp=%i (%f,%f,%f)  brp=%i (%f,%f,%f)  "
             "tlp=%i (%f,%f,%f)  trp=%i (%f,%f,%f)\n",
             blp, cloud_proj_->points[blp].x,
             cloud_proj_->points[blp].y, cloud_proj_->points[blp].z,
             brp, cloud_proj_->points[brp].x,
             cloud_proj_->points[brp].y, cloud_proj_->points[brp].z,
             tlp, cloud_proj_->points[tlp].x,
             cloud_proj_->points[tlp].y, cloud_proj_->points[tlp].z,
             trp, cloud_proj_->points[trp].x,
             cloud_proj_->points[trp].y, cloud_proj_->points[trp].z);

      pcl::Vertices v;
      v.vertices.push_back(blp);
      v.vertices.push_back(tlp);
      v.vertices.push_back(trp);
      v.vertices.push_back(brp);
      v.vertices.push_back(blp);
      vertices_.clear();
      vertices_.push_back(v);
    }
*/

    new_cloud_ = true;

    // To show differences between cloud_filt and cloud_above
    // (draw both, increase point size of cloud_above
    //for (int i = 0; i < cloud_filt_->points.size(); ++i) {
    //  cloud_filt_->points[i].r = 255;
    //  cloud_filt_->points[i].g =   0;
    //  cloud_filt_->points[i].b =   0;
    //}

    return (cloud_above_);
  }

  void
  viz_cb (pcl::visualization::PCLVisualizer& viz)
  {
    if (!new_cloud_)
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(1));
      return;
    }

    {
      boost::mutex::scoped_lock lock (mtx_);
      // Render the data 
      if (!viz.updatePointCloud(cloud_proj_, "table")) {
        viz.addPointCloud (cloud_proj_, "table");
      }
      if (!viz.updatePointCloud(cloud_objs_, "clusters")) {
        viz.addPointCloud (cloud_objs_, "clusters");
      }

      viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "table");
      viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "clusters");

      viz.removeShape ("hull");
      if (!vertices_.empty()) {
        viz.addPolygonMesh<PointType>(cloud_hull_, vertices_, "hull");
      }
      new_cloud_ = false;
    }
  }

  void
  run ()
  {
    /*
    pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

    boost::function<void (const CloudConstPtr&)> f =
      boost::bind (&OpenNIPlanarSegmentation::cloud_cb_, this, _1);
    boost::signals2::connection c = interface->registerCallback(f);
    */

    viewer.runOnVisualizationThread(boost::bind(&OpenNIPlanarSegmentation::viz_cb,
                                                this, _1), "viz_cb");

    SharedMemoryCamera cam("openni-pointcloud");
    SharedMemoryImageBuffer *buf = cam.shared_memory_image_buffer();
    //interface->start ();
      
    fawkes::Time last_update;

    while (!viewer.wasStopped ())
    {
      fawkes::Time ct = buf->capture_time();
      if (last_update != ct) {
        last_update = ct;

        cam.capture();

        CloudPtr cloud(new Cloud());
        convert_buffer_to_pcl(buf, *cloud);
        set(cloud);

        //the call to get() sets the cloud_ to null;
        viewer.showCloud(get());
      }
    }

    //interface->stop ();
  }

  pcl::visualization::CloudViewer viewer;
  pcl::VoxelGrid<PointType> grid_;
  pcl::SACSegmentation<PointType> seg_;
  pcl::ExtractIndices<PointType> extract_;
  std::vector<pcl::Vertices> vertices_;
  CloudPtr cloud_hull_;
  CloudPtr cloud_proj_;
  CloudPtr cloud_filt_;
  CloudPtr cloud_above_;
  CloudPtr cloud_objs_;
  pcl::KdTreeFLANN<PointType> kdtree_;

  std::string device_id_;
  boost::mutex mtx_;
  CloudConstPtr cloud_;
  bool new_cloud_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n"
            << "where options are:\n         -thresh X        :: set the planar segmentation threshold (default: 0.5)\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
           << ", connected: " << (int)driver.getBus (deviceIdx) << " @ " << (int)driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (wotks only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int 
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    usage (argv);
    return 1;
  }

  std::string arg (argv[1]);
  
  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return 1;
  }

  double threshold = 0.05;
  pcl::console::parse_argument (argc, argv, "-thresh", threshold);

  //pcl::OpenNIGrabber grabber (arg);
  //if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
  //{
    OpenNIPlanarSegmentation<pcl::PointXYZRGB> v (arg);
    v.run ();
    /*
  }
  else
  {
    printf("Only RGBD supported atm\n");
    //OpenNIPlanarSegmentation<pcl::PointXYZ> v (arg, threshold);
    //v.run ();
  }
    */

  return (0);
}

/// @endcond EXAMPLE
