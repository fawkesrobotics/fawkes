
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
#include <pcl/console/print.h>
#include <core/utils/refptr.h>
#include <utils/time/time.h>
#include <config/config.h>

namespace fawkes {
  namespace pcl_utils {
#if 0 /* just to make Emacs auto-indent happy */
  }
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


/** Call this function to make PCL shutup.
 * Warning: this makes PCL completely quiet for everything using
 * PCL in the same process.
 */
inline void
shutup()
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}


/** Shutup PCL based on configuration.
 * Check the configuration flag /pcl/shutup and call pcl_utils::shutup() if
 * it is set to true.
 * @param config config to query for value
 */
inline void
shutup_conditional(Configuration *config)
{
  bool pcl_shutup = false;
  try {
    pcl_shutup = config->get_bool("/pcl/shutup");
  } catch (Exception &e) {} // ignore, use default
  if (pcl_shutup)  ::fawkes::pcl_utils::shutup();
}


/** Set time of a point cloud from a fawkes::Time instance.
 * This uses the fawkes::PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to set the time
 * @param time time to use
 */
template <typename PointT>
inline void
set_time(pcl::PointCloud<PointT> &cloud, const fawkes::Time &time)
{
#if defined(HAVE_ROS_PCL) || defined(ROSCPP_TYPES_H)
  cloud.header.stamp.sec  = time.get_sec();
  cloud.header.stamp.nsec = time.get_usec() * 1000;
#else
#  if PCL_VERSION_COMPARE(>=,1,7,0)
  cloud.header.stamp = time.in_usec();
#  else
  PointCloudTimestamp pclts;
  pclts.time.sec  = time.get_sec();
  pclts.time.usec = time.get_usec();
  cloud.header.stamp = pclts.timestamp;
#  endif
#endif
}

/** Set time of a point cloud from a fawkes::Time instance.
 * This uses the PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to set the time
 * @param time time to use
 */
template <typename PointT>
inline void
set_time(fawkes::RefPtr<pcl::PointCloud<PointT> > &cloud, const fawkes::Time &time)
{
  set_time<PointT>(**cloud, time);
}

/** Set time of a point cloud from a fawkes::Time instance.
 * This uses the PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to set the time
 * @param time time to use
 */
template <typename PointT>
inline void
set_time(boost::shared_ptr<pcl::PointCloud<PointT> > &cloud, const fawkes::Time &time)
{
  set_time<PointT>(*cloud, time);
}



/** Get time of a point cloud as a fawkes::Time instance.
 * This uses the PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to get the time
 * @param time upon return contains the timestamp of the cloud
 */
template <typename PointT>
inline void
get_time(const fawkes::RefPtr<const pcl::PointCloud<PointT> > &cloud, fawkes::Time &time)
{
#if defined(HAVE_ROS_PCL) || defined(ROSCPP_TYPES_H)
  time.set_time(cloud->header.stamp.sec, cloud->header.stamp.nsec / 1000);
#else
#  if PCL_VERSION_COMPARE(>=,1,7,0)
  time.set_time(cloud->header.stamp / 1000000U, cloud->header.stamp % 1000000);
#  else
  PointCloudTimestamp pclts;
  pclts.timestamp = cloud->header.stamp;
  time.set_time(pclts.time.sec, pclts.time.usec);
#  endif
#endif
}


/** Get time of a point cloud as a fawkes::Time instance.
 * This uses the PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to get the time
 * @param time upon return contains the timestamp of the cloud
 */
template <typename PointT>
inline void
get_time(const fawkes::RefPtr<pcl::PointCloud<PointT> > &cloud, fawkes::Time &time)
{
#if defined(HAVE_ROS_PCL) || defined(ROSCPP_TYPES_H)
  time.set_time(cloud->header.stamp.sec, cloud->header.stamp.nsec / 1000);
#else
#  if PCL_VERSION_COMPARE(>=,1,7,0)
  time.set_time(cloud->header.stamp / 1000000U, cloud->header.stamp % 1000000);
#  else
  PointCloudTimestamp pclts;
  pclts.timestamp = cloud->header.stamp;
  time.set_time(pclts.time.sec, pclts.time.usec);
#endif
#endif
}


/** Get time of a point cloud as a fawkes::Time instance.
 * This uses the PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to get the time
 * @param time upon return contains the timestamp of the cloud
 */
template <typename PointT>
inline void
get_time(const pcl::PointCloud<PointT> &cloud, fawkes::Time &time)
{
#if defined(HAVE_ROS_PCL) || defined(ROSCPP_TYPES_H)
  time.set_time(cloud.header.stamp.sec, cloud.header.stamp.nsec / 1000);
#else
#  if PCL_VERSION_COMPARE(>=,1,7,0)
  time.set_time(cloud.header.stamp / 1000000U, cloud.header.stamp % 1000000);
#  else
  PointCloudTimestamp pclts;
  pclts.timestamp = cloud.header.stamp;
  time.set_time(pclts.time.sec, pclts.time.usec);
#  endif
#endif
}


/** Get time of a point cloud as a fawkes::Time instance.
 * This uses the PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to get the time
 * @param time upon return contains the timestamp of the cloud
 */
template <typename PointT>
inline void
get_time(const boost::shared_ptr<pcl::PointCloud<PointT> > &cloud, fawkes::Time &time)
{
#if defined(HAVE_ROS_PCL) || defined(ROSCPP_TYPES_H)
  time.set_time(cloud->header.stamp.sec, cloud->header.stamp.nsec / 1000);
#else
#  if PCL_VERSION_COMPARE(>=,1,7,0)
  time.set_time(cloud->header.stamp / 1000000U, cloud->header.stamp % 1000000);
#  else
  PointCloudTimestamp pclts;
  pclts.timestamp = cloud->header.stamp;
  time.set_time(pclts.time.sec, pclts.time.usec);
#  endif
#endif
}


/** Get time of a point cloud as a fawkes::Time instance.
 * This uses the PointCloudTimestamp struct to set the time in the PCL
 * timestamp field (if non-ROS PCL is used).
 * @param cloud cloud of which to get the time
 * @param time upon return contains the timestamp of the cloud
 */
template <typename PointT>
inline void
get_time(const boost::shared_ptr<const pcl::PointCloud<PointT>> &cloud, fawkes::Time &time)
{
#if defined(HAVE_ROS_PCL) || defined(ROSCPP_TYPES_H)
  time.set_time(cloud->header.stamp.sec, cloud->header.stamp.nsec / 1000);
#else
#  if PCL_VERSION_COMPARE(>=,1,7,0)
  time.set_time(cloud->header.stamp / 1000000U, cloud->header.stamp % 1000000);
#  else
  PointCloudTimestamp pclts;
  pclts.timestamp = cloud->header.stamp;
  time.set_time(pclts.time.sec, pclts.time.usec);
#  endif
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


/** Copy time from one point cloud to another.
 * @param from point cloud to copy time from
 * @param to point cloud to copy time to
 */
template <typename PointT1, typename PointT2>
inline void
copy_time(boost::shared_ptr<const pcl::PointCloud<PointT1> > &from,
	  fawkes::RefPtr<pcl::PointCloud<PointT2> > &to)
{
  to->header.stamp = from->header.stamp;
}

/** Copy time from one point cloud to another.
 * @param from point cloud to copy time from
 * @param to point cloud to copy time to
 */
template <typename PointT1, typename PointT2>
inline void
copy_time(const pcl::PointCloud<PointT1> &from,
	  pcl::PointCloud<PointT2> &to)
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
cloudptr_from_refptr(const fawkes::RefPtr<pcl::PointCloud<PointT> > &in)
{
  return
    boost::shared_ptr<pcl::PointCloud<PointT> >(*in, PointCloudNonDeleter());
}


template <typename PointT>
typename pcl::PointCloud<PointT>::ConstPtr
cloudptr_from_refptr(const fawkes::RefPtr<const pcl::PointCloud<PointT> > &in)
{
  return
    boost::shared_ptr<const pcl::PointCloud<PointT> >(*in, PointCloudNonDeleter());
}

} // end namespace pclutils
} // end namespace fawkes

#endif
