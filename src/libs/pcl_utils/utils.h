
/***************************************************************************
 *  utils.h - General PCL utilities
 *
 *  Created: Tue Nov 08 17:50:07 2011
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

#ifndef __LIBS_PCL_UTILS_UTILS_H_
#define __LIBS_PCL_UTILS_UTILS_H_

#include <pcl/point_cloud.h>

namespace fawkes {
  namespace pcl_utils {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** Set time of a point cloud from a fawkes::Time instance.
 * This uses the fawkes::PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to set the time
 * @param time time to use
 */
template <typename PointT>
inline void
set_time(fawkes::RefPtr<pcl::PointCloud<PointT> > &cloud, const fawkes::Time &time)
{
#if HAVE_ROS_PCL
  cloud->header.stamp.sec  = time.get_sec();
  cloud->header.stamp.nsec = time.get_usec() * 1000;
#else
  fawkes::PointCloudTimestamp pclts;
  pclts.time.sec  = time.get_sec();
  pclts.time.usec = time.get_usec();
  cloud->header.stamp = pclts.timestamp;
#endif
}


/** Get time of a point cloud as a fawkes::Time instance.
 * This uses the fawkes::PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to get the time
 * @param time upon return contains the timestamp of the cloud
 */
template <typename PointT>
inline void
get_time(const fawkes::RefPtr<const pcl::PointCloud<PointT> > &cloud, fawkes::Time &time)
{
#if HAVE_ROS_PCL
  time.set_time(cloud->header.stamp.sec, cloud->header.stamp.nsec / 1000);
#else
  fawkes::PointCloudTimestamp pclts;
  pclts.timestamp = cloud->header.stamp;
  time.set_time(pclts.time.sec, time.get_usec());
#endif
}


/** Get time of a point cloud as a fawkes::Time instance.
 * This uses the fawkes::PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to get the time
 * @param time upon return contains the timestamp of the cloud
 */
template <typename PointT>
inline void
get_time(const fawkes::RefPtr<pcl::PointCloud<PointT> > &cloud, fawkes::Time &time)
{
#if HAVE_ROS_PCL
  time.set_time(cloud->header.stamp.sec, cloud->header.stamp.nsec / 1000);
#else
  fawkes::PointCloudTimestamp pclts;
  pclts.timestamp = cloud->header.stamp;
  time.set_time(pclts.time.sec, time.get_usec());
#endif
}


/** Copy time from one point cloud to another.
 * @param from point cloud to copy time from
 * @param to point cloud to copy time to
 */
template <typename PointT1, typename PointT2>
inline void
copy_time(fawkes::RefPtr<const pcl::PointCloud<PointT1> > &from,
          fawkes::RefPtr<pcl::PointCloud<PointT2> > &to)
{
  to->header.stamp = from->header.stamp;
}


/** Helper struct to avoid deletion of PointClouds.
 * The input point cloud is accessible using a RefPtr. Since the PCL
 * expectes Boost shared_ptr, we need to create such a shared pointer.
 * But destruction of this would cause the deletion of the point cloud,
 * which we do not want. Therefore, we provide this helper deleter
 * that causes the PointCloud *not* to be deleted on reset.
 */
struct PointCloudNonDeleter {
  /** Delete operator that does nothing.
   * @param t object to destroy
   */
  template<typename T> void operator()(T*t) {}
};

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
cloudptr_from_refptr(fawkes::RefPtr<pcl::PointCloud<PointT> > &in)
{
  return
    boost::shared_ptr<pcl::PointCloud<PointT> >(*in, PointCloudNonDeleter());
}


template <typename PointT>
typename pcl::PointCloud<PointT>::ConstPtr
cloudptr_from_refptr(fawkes::RefPtr<const pcl::PointCloud<PointT> > &in)
{
  return
    boost::shared_ptr<const pcl::PointCloud<PointT> >(*in, PointCloudNonDeleter());
}

} // end namespace pclutils
} // end namespace fawkes

#endif
