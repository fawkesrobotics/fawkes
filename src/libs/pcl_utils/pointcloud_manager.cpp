
/***************************************************************************
 *  pointcloud_manager.cpp - PointCloud manager
 *
 *  Created: Sun Nov 06 23:49:36 2011
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <pcl_utils/pointcloud_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class PointCloudManager <pcl_utils/pointcloud_manager.h>
 * Point Cloud manager.
 * This class manages a number of points clouds and acts as a hub to
 * distribute them.
 * @author Tim Niemueller
 *
 * @fn void PointCloudManager::add_pointcloud(const char *id, RefPtr<pcl::PointCloud<PointT> > cloud)
 * Add point cloud.
 * @param id ID of point cloud to add, must be unique
 * @param cloud refptr to point cloud
 *
 * @fn const RefPtr<const pcl::PointCloud<PointT> > PointCloudManager::get_pointcloud(const char *id)
 * Get point cloud.
 * @param id ID of point cloud to retrieve
 * @return point cloud
 * @exception Exception thrown if point cloud for given ID does not exist
 *
 */

/** Constructor. */
PointCloudManager::PointCloudManager()
{
}

/** Destructor. */
PointCloudManager::~PointCloudManager()
{
  LockMap<std::string, pcl_utils::StorageAdapter *>::iterator c;
  for (c = __clouds.begin(); c != __clouds.end(); ++c) {
    delete c->second;
  }

  __clouds.clear();
}


/** Remove the point cloud.
 * @param id ID of point cloud to remove
 */
void
PointCloudManager::remove_pointcloud(const char *id)
{
  MutexLocker lock(__clouds.mutex());

  if (__clouds.find(id) != __clouds.end()) {
    delete __clouds[id];
    __clouds.erase(id);
  }
}

/** Check if point cloud exists
 * @param id ID of point cloud to check
 * @return true if the point cloud exists, false otherwise
 */
bool
PointCloudManager::exists_pointcloud(const char *id)
{
  MutexLocker lock(__clouds.mutex());

  return (__clouds.find(id) != __clouds.end());
}


/** Get list of point cloud IDs.
 * @return list of point cloud IDs
 */
std::vector<std::string>
PointCloudManager::get_pointcloud_list() const
{
  MutexLocker lock(__clouds.mutex());

  std::vector<std::string> rv;
  rv.clear();
  LockMap<std::string, pcl_utils::StorageAdapter *>::const_iterator c;
  for (c = __clouds.begin(); c != __clouds.end(); ++c) {
    rv.push_back(c->first);
  }
  return rv;
}


/** Get map of point clouds.
 * Use with care. Do not use in ROS-enabled plugins unless you are aware
 * of sensor_msgs and std_msgs incompatibilities between standalone PCL
 * and ROS!
 * @return map from ID to storage adapter
 */
const fawkes::LockMap<std::string, pcl_utils::StorageAdapter *> &
PointCloudManager::get_pointclouds() const
{
  return __clouds;
}


/** Get a storage adapter.
 * Use with care. Do not use in ROS-enabled plugins unless you are aware
 * of sensor_msgs and std_msgs incompatibilities between standalone PCL
 * and ROS!
 * @param id ID of point clouds whose storage adapter to retrieve
 * @return storage adapter for given ID
 * @exception Exception thrown if ID is unknown
 */
const pcl_utils::StorageAdapter *
PointCloudManager::get_storage_adapter(const char *id)
{
  MutexLocker lock(__clouds.mutex());

  if (__clouds.find(id) == __clouds.end()) {
    throw Exception("PointCloud '%s' unknown", id);
  }
  return __clouds[id];
}



} // end namespace fawkes
