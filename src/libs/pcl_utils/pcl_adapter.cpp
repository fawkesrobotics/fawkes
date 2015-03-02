
/***************************************************************************
 *  pcl_adapter.cpp - PCL exchange publisher manager
 *
 *  Created: Tue Nov 08 00:38:34 2011
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
 *             2012       Bastian Klingen
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

#include "pcl_adapter.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#if PCL_VERSION_COMPARE(>=,1,7,0)
#  include <pcl/PCLPointField.h>
#endif
#include <logging/logger.h>
#include <pcl_utils/pointcloud_manager.h>

using namespace fawkes;

/// @cond INTERNALS
/** ROS to PCL storage adapter. */
class PointCloudAdapter::StorageAdapter
{
 public:
  /** Constructor.
   * @param a_ adapter to clone */
  StorageAdapter(const pcl_utils::StorageAdapter *a_)
    : a(a_->clone()) {}

  /** Destructor. */
  ~StorageAdapter()
  { delete a; }

  /** PCL Point cloud storage adapter to encapsulate. */ 
  pcl_utils::StorageAdapter *a;
};
/// @endcond

/** @class PointCloudAdapter <pcl_utils/pcl_adapter.h>
 * Point cloud adapter class.
 * Currently, the standalone PCL comes with sensor_msgs and std_msgs
 * data types which are incompatible with the ones that come with
 * ROS. Hence, to use both in the same plugin, we need to confine the
 * two different type instances into their own modules. While the rest
 * of the ros-pcl plugins uses ROS-specific data types, this very
 * class interacts with and only with the standalone PCL and the
 * PointCloudManager. Interaction to the ROS parts is done by passing
 * internal data types so that exchange can happen without common
 * sensor_msgs or std_msgs data types.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param pcl_manager PCL manager
 * @param logger logger
 */
PointCloudAdapter::PointCloudAdapter(PointCloudManager *pcl_manager,
                                           Logger *logger)
  : __pcl_manager(pcl_manager)
{
}


/** Destructor. */
PointCloudAdapter::~PointCloudAdapter()
{
  std::map<std::string, StorageAdapter *>::iterator i;
  for (i = __sas.begin(); i != __sas.end(); ++i) {
    delete i->second;
  }
  __sas.clear();
}

/** Fill information of arbitrary point type.
 * @param p point cloud to get info from
 * @param width upon return contains width of point cloud
 * @param height upon return contains width of point cloud
 * @param frame_id upon return contains frame ID of the point cloud
 * @param is_dense upon return contains true if point cloud is dense and false otherwise
 * @param pfi upon return contains data type information
 */
template <typename PointT>
static void
fill_info(const fawkes::RefPtr<const pcl::PointCloud<PointT> > &p,
          unsigned int &width, unsigned int &height,
          std::string &frame_id, bool &is_dense,
          PointCloudAdapter::V_PointFieldInfo &pfi)
{
  width  = p->width;
  height = p->height;
  frame_id = p->header.frame_id;
  is_dense = p->is_dense;

#if PCL_VERSION_COMPARE(>=,1,7,0)
  std::vector<pcl::PCLPointField> pfields;
#else
  std::vector<sensor_msgs::PointField> pfields;
#endif
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>
    (pcl::detail::FieldAdder<PointT>(pfields));

  pfi.clear();
  pfi.resize(pfields.size());
  for (unsigned int i = 0; i < pfields.size(); ++i) {
#if PCL_VERSION_COMPARE(>=,1,7,0)
    pcl::PCLPointField &pf = pfields[i];
#else
    sensor_msgs::PointField &pf = pfields[i];
#endif
    pfi[i] = PointCloudAdapter::PointFieldInfo(pf.name, pf.offset,
                                                  pf.datatype, pf.count);
  }
}
                 
/** Get info about point cloud.
 * @param id ID of point cloud to get info from
 * @param width upon return contains width of point cloud
 * @param height upon return contains width of point cloud
 * @param frame_id upon return contains frame ID of the point cloud
 * @param is_dense upon return contains true if point cloud is dense and false otherwise
 * @param pfi upon return contains data type information
 */
void
PointCloudAdapter::get_info(const std::string &id,
			    unsigned int &width, unsigned int &height,
			    std::string &frame_id, bool &is_dense,
			    V_PointFieldInfo &pfi)
{
  if (__sas.find(id) == __sas.end()) {
    __sas[id] = new StorageAdapter(__pcl_manager->get_storage_adapter(id.c_str()));
  }

  if (__pcl_manager->exists_pointcloud<pcl::PointXYZ>(id.c_str())) {
    const fawkes::RefPtr<const pcl::PointCloud<pcl::PointXYZ> > p =
      __pcl_manager->get_pointcloud<pcl::PointXYZ>(id.c_str());
    fill_info(p, width, height, frame_id, is_dense, pfi);

  } else if (__pcl_manager->exists_pointcloud<pcl::PointXYZRGB>(id.c_str())) {
    const fawkes::RefPtr<const pcl::PointCloud<pcl::PointXYZRGB> > p =
      __pcl_manager->get_pointcloud<pcl::PointXYZRGB>(id.c_str());
    fill_info(p, width, height, frame_id, is_dense, pfi);

  } else if (__pcl_manager->exists_pointcloud<pcl::PointXYZL>(id.c_str())) {
    const fawkes::RefPtr<const pcl::PointCloud<pcl::PointXYZL> > p =
      __pcl_manager->get_pointcloud<pcl::PointXYZL>(id.c_str());
    fill_info(p, width, height, frame_id, is_dense, pfi);

  } else {
    throw Exception("PointCloud '%s' does not exist or unknown type", id.c_str());
  }
}


/** Get current data of point cloud.
 * @param id ID of point cloud to get info from
 * @param frame_id upon return contains the frame ID of the point cloud
 * @param width upon return contains width of point cloud
 * @param height upon return contains width of point cloud
 * @param time capture time
 * @param data_ptr upon return contains pointer to point cloud data
 * @param point_size upon return contains size of a single point
 * @param num_points upon return contains number of points
 */
void
PointCloudAdapter::get_data(const std::string &id, std::string &frame_id,
                               unsigned int &width, unsigned int &height, fawkes::Time &time,
                               void **data_ptr, size_t &point_size, size_t &num_points)
{
  if (__sas.find(id) == __sas.end()) {
    __sas[id] = new StorageAdapter(__pcl_manager->get_storage_adapter(id.c_str()));
  }

  const pcl_utils::StorageAdapter *sa = __sas[id]->a;
  frame_id = sa->frame_id();
  width  = sa->width();
  height = sa->height();
  *data_ptr = sa->data_ptr();
  point_size = sa->point_size();
  num_points = sa->num_points();
  sa->get_time(time);
}


/** Get data and info of point cloud.
 * @param id ID of point cloud to get info from
 * @param frame_id upon return contains frame ID of the point cloud
 * @param is_dense upon return contains true if point cloud is dense and false otherwise
 * @param width upon return contains width of point cloud
 * @param height upon return contains width of point cloud
 * @param time capture time for which to get the data and info
 * @param pfi upon return contains data type information
 * @param data_ptr upon return contains pointer to point cloud data
 * @param point_size upon return contains size of a single point
 * @param num_points upon return contains number of points
 */
void
PointCloudAdapter::get_data_and_info(const std::string &id, std::string &frame_id, bool &is_dense,
				     unsigned int &width, unsigned int &height, fawkes::Time &time,
				     V_PointFieldInfo &pfi,
				     void **data_ptr, size_t &point_size, size_t &num_points)
{
  get_info(id, width, height, frame_id, is_dense, pfi);
  get_data(id, frame_id, width, height, time, data_ptr, point_size, num_points);
}


/** Close an adapter.
 * @param id ID of point cloud to close
 */
void
PointCloudAdapter::close(const std::string &id)
{
  if (__sas.find(id) != __sas.end()) {
    delete __sas[id];
    __sas.erase(id);
  }
}
