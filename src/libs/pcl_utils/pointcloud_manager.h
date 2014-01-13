
/***************************************************************************
 *  pointcloud_manager.h - PointCloud manager for aspect
 *
 *  Created: Sun Nov 06 23:29:36 2011
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

#ifndef __LIBS_PCL_UTILS_POINTCLOUD_MANAGER_H_
#define __LIBS_PCL_UTILS_POINTCLOUD_MANAGER_H_

#include <core/exception.h>
#include <core/utils/refptr.h>
#include <core/utils/lock_map.h>
#include <core/threading/mutex_locker.h>
#include <utils/time/time.h>

#include <pcl_utils/storage_adapter.h>

#include <vector>
#include <string>
#include <stdint.h>
#include <typeinfo>
#include <cstring>

namespace pcl {
  template <typename PointT>
    class PointCloud;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class PointCloudManager
{
 public:
  PointCloudManager();
  virtual ~PointCloudManager();

  template <typename PointT>
    void add_pointcloud(const char *id, RefPtr<pcl::PointCloud<PointT> > cloud);

  void remove_pointcloud(const char *id);

  template <typename PointT>
    const RefPtr<const pcl::PointCloud<PointT> > get_pointcloud(const char *id);
  bool exists_pointcloud(const char *id);

  /**  Check if point cloud of specified type exists.
   * @param id ID of point cloud to check
   * @return true if the point cloud exists, false otherwise
   */
  template <typename PointT>
  bool exists_pointcloud(const char *id);


  std::vector<std::string>  get_pointcloud_list() const;
  const fawkes::LockMap<std::string, pcl_utils::StorageAdapter *> &  get_pointclouds() const;
  const pcl_utils::StorageAdapter *  get_storage_adapter(const char *id);

 private:
  fawkes::LockMap<std::string, pcl_utils::StorageAdapter *>  __clouds;
};


template <typename PointT>
void
PointCloudManager::add_pointcloud(const char *id,
                                  RefPtr<pcl::PointCloud<PointT> > cloud)
{
  fawkes::MutexLocker lock(__clouds.mutex());

  if (__clouds.find(id) == __clouds.end()) {
    __clouds[id] = new pcl_utils::PointCloudStorageAdapter<PointT>(cloud);
  } else {
    throw Exception("Cloud %s already registered");
  }
}

template <typename PointT>
const RefPtr<const pcl::PointCloud<PointT> >
PointCloudManager::get_pointcloud(const char *id)
{
  fawkes::MutexLocker lock(__clouds.mutex());

  if (__clouds.find(id) != __clouds.end()) {
    pcl_utils::PointCloudStorageAdapter<PointT> *pa =
      dynamic_cast<pcl_utils::PointCloudStorageAdapter<PointT> *>(__clouds[id]);

    if (!pa) {
      // workaround for older compilers
      if (strcmp(__clouds[id]->get_typename(),
                 typeid(pcl_utils::PointCloudStorageAdapter<PointT> *).name()) == 0)
      {
        return static_cast<pcl_utils::PointCloudStorageAdapter<PointT> *>(__clouds[id])->cloud;
      }

      throw Exception("The desired point cloud is of a different type");
    }
    return pa->cloud;
  } else {
    throw Exception("No point cloud with ID '%s' registered", id);
  }
}

template <typename PointT>
bool
PointCloudManager::exists_pointcloud(const char *id)
{
  try {
    const RefPtr<const pcl::PointCloud<PointT> > p = get_pointcloud<PointT>(id);
    return true;
  } catch (Exception &e) {
    return false;
  }

}

} // end namespace fawkes

#endif
