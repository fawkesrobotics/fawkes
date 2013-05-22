
/***************************************************************************
 *  storage_adapter.h - Utility to store differently typed point clouds in
 *                      common container.
 *
 *  Created: Fri Nov 30 16:40:51 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_PCL_UTILS_STORAGE_ADAPTER_H_
#define __LIBS_PCL_UTILS_STORAGE_ADAPTER_H_

#include <pcl_utils/utils.h>
#include <pcl_utils/transforms.h>
#include <pcl/point_cloud.h>

namespace fawkes {
  namespace pcl_utils {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

template <typename PointT>
class PointCloudStorageAdapter;

class StorageAdapter {
 public:
  /** Virtual empty destructor. */
  virtual ~StorageAdapter() {};

  template <typename PointT>
  bool is_pointtype() const;

  template <typename PointT>
  PointCloudStorageAdapter<PointT> * as_pointtype();

  virtual void transform(const std::string &target_frame,
			 const tf::Transformer &transformer) = 0;

  virtual void transform(const std::string &target_frame,
			 const Time &target_time,
			 const std::string &fixed_frame,
			 const tf::Transformer &transformer) = 0;

  virtual const char * get_typename() = 0;
  virtual StorageAdapter * clone() const = 0;
  virtual size_t  point_size() const = 0;
  virtual unsigned int  width() const = 0;
  virtual unsigned int  height() const = 0;
  virtual size_t  num_points() const = 0;
  virtual void *  data_ptr() const = 0;
  virtual std::string frame_id() const = 0;
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

  /** Get PCL shared pointer to cloud.
   * @return PCL shared pointer to cloud
   */
  typename pcl::PointCloud<PointT>::Ptr cloud_ptr()
  { return pcl_utils::cloudptr_from_refptr(cloud); }

  /** Get PCL const shared pointer to cloud.
   * @return PCL const shared pointer to cloud
   */
  typename pcl::PointCloud<PointT>::ConstPtr cloud_const_ptr()
  { return pcl_utils::cloudptr_from_refptr(cloud); }

  virtual StorageAdapter * clone() const;

  virtual void transform(const std::string &target_frame,
			 const tf::Transformer &transformer);

  virtual void transform(const std::string &target_frame,
			 const Time &target_time,
			 const std::string &fixed_frame,
			 const tf::Transformer &transformer);

  virtual const char * get_typename() { return typeid(this).name(); }
  virtual size_t  point_size() const { return sizeof(PointT); }
  virtual unsigned int  width() const { return cloud->width; }
  virtual unsigned int  height() const { return cloud->height; }
  virtual size_t  num_points() const { return cloud->points.size(); }
  virtual void *  data_ptr() const  { return &cloud->points[0]; }
  virtual std::string frame_id() const { return cloud->header.frame_id; }
  virtual void get_time(fawkes::Time &time) const;
};

template <typename PointT>
bool
StorageAdapter::is_pointtype() const
{
  const PointCloudStorageAdapter<PointT> *pa =
    dynamic_cast<const PointCloudStorageAdapter<PointT> *>(this);
  return (!!pa);
}


template <typename PointT>
PointCloudStorageAdapter<PointT> *
StorageAdapter::as_pointtype()
{
  PointCloudStorageAdapter<PointT> *pa =
    dynamic_cast<PointCloudStorageAdapter<PointT> *>(this);
  if (!pa) {
    throw Exception("PointCloud storage adapter is not of anticipated type");
  }
  return pa;
}


template <typename PointT>
StorageAdapter *
PointCloudStorageAdapter<PointT>::clone() const
{
  return new PointCloudStorageAdapter<PointT>(this);
}


template <typename PointT>
void
PointCloudStorageAdapter<PointT>::get_time(fawkes::Time &time) const
{
  pcl_utils::get_time(cloud, time);
}


template <typename PointT>
void
PointCloudStorageAdapter<PointT>::transform(const std::string &target_frame,
					    const tf::Transformer &transformer)
{
  pcl::PointCloud<PointT> tmp;
  pcl_utils::transform_pointcloud(target_frame, **cloud, tmp, transformer);
  **cloud = tmp;
}

template <typename PointT>
void
PointCloudStorageAdapter<PointT>::transform(const std::string &target_frame,
					    const Time &target_time,
					    const std::string &fixed_frame,
					    const tf::Transformer &transformer)
{
  pcl::PointCloud<PointT> tmp;
  pcl_utils::transform_pointcloud(target_frame, target_time, fixed_frame,
				  **cloud, tmp, transformer);
  **cloud = tmp;
}



} // end namespace pcl_utils
} // end namespace fawkes

#endif
