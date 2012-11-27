
/***************************************************************************
 *  pcl_adapter.cpp - Adapter to receive information about point clouds
 *
 *  Created: Tue Nov 08 00:38:34 2011
 *  Copyright  2010-2012  Tim Niemueller [www.niemueller.de]
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
#include <pcl/point_representation.h>
#include <pcl/common/io.h>
#include <logging/logger.h>
#include <aspect/pointcloud/pointcloud_manager.h>

using namespace fawkes;

/// @cond INTERNALS
/** ROS to PCL storage adapter. */
class MongoLogPointCloudAdapter::StorageAdapter
{
 public:
  /** Constructor.
   * @param a_ adapter to clone */
  StorageAdapter(const PointCloudManager::StorageAdapter *a_)
    : a(a_->clone()) {}

  /** Destructor. */
  ~StorageAdapter()
  { delete a; }

  /** PCL Point cloud storage adapter to encapsulate. */ 
  PointCloudManager::StorageAdapter *a;
};
/// @endcond

/** @class MongoLogPointCloudAdapter "pcl_adapter.h"
 * Used to get additional information about point clouds.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param pcl_manager PCL manager
 * @param logger logger
 */
MongoLogPointCloudAdapter::MongoLogPointCloudAdapter(PointCloudManager *pcl_manager,
                                           Logger *logger)
  : __pcl_manager(pcl_manager)
{
}


/** Destructor. */
MongoLogPointCloudAdapter::~MongoLogPointCloudAdapter()
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
          MongoLogPointCloudAdapter::V_PointFieldInfo &pfi)
{
  width  = p->width;
  height = p->height;
  frame_id = p->header.frame_id;
  is_dense = p->is_dense;

  std::vector<sensor_msgs::PointField> pfields;
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>
    (pcl::detail::FieldAdder<PointT>(pfields));

  pfi.clear();
  pfi.resize(pfields.size());
  for (unsigned int i = 0; i < pfields.size(); ++i) {
    sensor_msgs::PointField &pf = pfields[i];
    pfi[i] = MongoLogPointCloudAdapter::PointFieldInfo(pf.name, pf.offset,
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
MongoLogPointCloudAdapter::get_info(std::string &id,
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

  } else {
    throw Exception("PointCloud '%s' does not exist or unknown type", id.c_str());
  }
}


/** Get current data of point cloud.
 * @param id ID of point cloud to get info from
 * @param width upon return contains width of point cloud
 * @param height upon return contains width of point cloud
 * @param time capture time
 * @param data_ptr upon return contains pointer to point cloud data
 * @param point_size upon return contains size of a single point
 * @param num_points upon return contains number of points
 */
void
MongoLogPointCloudAdapter::get_data(const std::string &id,
                               unsigned int &width, unsigned int &height, fawkes::Time &time,
                               void **data_ptr, size_t &point_size, size_t &num_points)
{
  if (__sas.find(id) == __sas.end()) {
    __sas[id] = new StorageAdapter(__pcl_manager->get_storage_adapter(id.c_str()));
  }

  const PointCloudManager::StorageAdapter *sa = __sas[id]->a;
  width  = sa->width();
  height = sa->height();
  *data_ptr = sa->data_ptr();
  point_size = sa->point_size();
  num_points = sa->num_points();
  sa->get_time(time);
}


/** Close an adapter.
 * @param id ID of point cloud to close
 */
void
MongoLogPointCloudAdapter::close(const std::string &id)
{
  if (__sas.find(id) != __sas.end()) {
    delete __sas[id];
    __sas.erase(id);
  }
}
