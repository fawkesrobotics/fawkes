
/***************************************************************************
 *  pointcloud_manager.h - PointCloud manager for aspect
 *
 *  Created: Sun Nov 06 23:29:36 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __ASPECT_POINTCLOUD_POINTCLOUD_MANAGER_H_
#define __ASPECT_POINTCLOUD_POINTCLOUD_MANAGER_H_

#include <aspect/aspect.h>
#include <core/exception.h>
#include <core/utils/refptr.h>
#include <core/utils/lock_map.h>
#include <core/threading/mutex_locker.h>
#include <utils/time/time.h>

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

/** Union to pack fawkes::Time into the pcl::PointCloud timestamp. */
typedef union {
  struct {
    uint64_t sec  : 44;	///< seconds part of time
    uint64_t usec : 20;	///< microseconds part of time
  } time;		///< Access timestamp as time
  uint64_t timestamp;	///< Access timestamp as number only
} PointCloudTimestamp;


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

  template <typename PointT>
    class PointCloudStorageAdapter;

  class StorageAdapter {
  public:
    virtual ~StorageAdapter();

    template <typename PointT>
    bool is_pointtype() const;

    template <typename PointT>
    PointCloudStorageAdapter<PointT> * as_pointtype();

    virtual const char * get_typename() = 0;
    virtual StorageAdapter * clone() const = 0;
    virtual size_t  point_size() const = 0;
    virtual unsigned int  width() const = 0;
    virtual unsigned int  height() const = 0;
    virtual size_t  num_points() const = 0;
    virtual void *  data_ptr() const = 0;
    virtual void get_time(fawkes::Time &time) const = 0;
  };

  template <typename PointT>
    class PointCloudStorageAdapter : public StorageAdapter
  {
   public:
    /** Constructor.
     * @param cloud cloud to encapsulate.
     */
    PointCloudStorageAdapter(RefPtr<pcl::PointCloud<PointT> > cloud)
      : cloud(cloud) {}

    /** Copy constructor.
     * @param p storage adapter to copy
     */
    PointCloudStorageAdapter(const PointCloudStorageAdapter<PointT> *p)
      : cloud(p->cloud) {}

    /** The point cloud. */
    const RefPtr<pcl::PointCloud<PointT> > cloud;

    virtual StorageAdapter * clone() const;

    virtual const char * get_typename() { return typeid(this).name(); }
    virtual size_t  point_size() const { return sizeof(PointT); }
    virtual unsigned int  width() const { return cloud->width; }
    virtual unsigned int  height() const { return cloud->height; }
    virtual size_t  num_points() const { return cloud->points.size(); }
    virtual void *  data_ptr() const  { return &cloud->points[0]; }
    virtual void get_time(fawkes::Time &time) const;
  };

  std::vector<std::string>  get_pointcloud_list() const;
  const fawkes::LockMap<std::string, StorageAdapter *> &  get_pointclouds() const;
  const StorageAdapter *  get_storage_adapter(const char *id);

 private:
  fawkes::LockMap<std::string, StorageAdapter *>  __clouds;
};


template <typename PointT>
bool
PointCloudManager::StorageAdapter::is_pointtype() const
{
  const PointCloudStorageAdapter<PointT> *pa =
    dynamic_cast<const PointCloudStorageAdapter<PointT> *>(this);
  return (!!pa);
}


template <typename PointT>
PointCloudManager::PointCloudStorageAdapter<PointT> *
PointCloudManager::StorageAdapter::as_pointtype()
{
  PointCloudStorageAdapter<PointT> *pa =
    dynamic_cast<PointCloudStorageAdapter<PointT> *>(this);
  if (!pa) {
    throw Exception("PointCloud storage adapter is not of anticipated type");
  }
  return pa;
}

template <typename PointT>
void
PointCloudManager::add_pointcloud(const char *id,
                                  RefPtr<pcl::PointCloud<PointT> > cloud)
{
  fawkes::MutexLocker lock(__clouds.mutex());

  if (__clouds.find(id) == __clouds.end()) {
    __clouds[id] = new PointCloudStorageAdapter<PointT>(cloud);
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
    PointCloudStorageAdapter<PointT> *pa =
      dynamic_cast<PointCloudStorageAdapter<PointT> *>(__clouds[id]);

    if (!pa) {
      // workaround for older compilers
      if (strcmp(__clouds[id]->get_typename(),
                 typeid(PointCloudStorageAdapter<PointT> *).name()) == 0)
      {
        return static_cast<PointCloudStorageAdapter<PointT> *>(__clouds[id])->cloud;
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

template <typename PointT>
PointCloudManager::StorageAdapter *
PointCloudManager::PointCloudStorageAdapter<PointT>::clone() const
{
  return new PointCloudStorageAdapter<PointT>(this);
}


template <typename PointT>
void
PointCloudManager::PointCloudStorageAdapter<PointT>::get_time(fawkes::Time &time) const
{
#if defined(HAVE_ROS_PCL) || defined(ROSCPP_TYPES_H)
  time.set_time(cloud->header.stamp.sec, cloud->header.stamp.nsec / 1000);
#else
  fawkes::PointCloudTimestamp pclts;
  pclts.timestamp = cloud->header.stamp;
  time.set_time(pclts.time.sec, time.get_usec());
#endif
}



} // end namespace fawkes

#endif
