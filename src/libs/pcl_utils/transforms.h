
/***************************************************************************
 *  transforms.h - PCL utilities: apply transforms to point clouds
 *
 *  Created: Fri Nov 30 13:33:40 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
 *             2010  Willow Garage, Inc.
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

#ifndef __LIBS_PCL_UTILS_TRANSFORMS_H_
#define __LIBS_PCL_UTILS_TRANSFORMS_H_

#include <tf/types.h>
#include <tf/transformer.h>
#include <pcl_utils/utils.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

namespace fawkes {
  namespace pcl_utils {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** Apply a rigid transform.
 * @param cloud_in the input point cloud
 * @param cloud_out the input point cloud
 * @param transform a rigid transformation from tf
 * @note calls the Eigen version
 */
template <typename PointT> void 
transform_pointcloud(const pcl::PointCloud<PointT> &cloud_in, 
		     pcl::PointCloud<PointT> &cloud_out, 
		     const tf::Transform &transform)
{
  // Bullet (used by tf) and Eigen both store quaternions in x,y,z,w
  // order, despite the ordering of arguments in Eigen's
  // constructor. We could use an Eigen Map to convert without copy,
  // but this only works if Bullet uses floats, that is if
  // BT_USE_DOUBLE_PRECISION is not defined.  Rather that risking a
  // mistake, we copy the quaternion, which is a small cost compared
  // to the conversion of the point cloud anyway. Idem for the origin.

  tf::Quaternion q = transform.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z()); // internally stored as (x,y,z,w)
  tf::Vector3 v = transform.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());
  pcl::transformPointCloud(cloud_in, cloud_out, origin, rotation);
}


/** Apply a rigid transform.
 * @param cloud_inout input and output point cloud
 * @param transform a rigid transformation from tf
 * @note calls the Eigen version
 */
template <typename PointT> void 
transform_pointcloud(pcl::PointCloud<PointT> &cloud_inout, 
		     const tf::Transform &transform)
{
  pcl::PointCloud<PointT> tmp;
  transform_pointcloud(cloud_inout, tmp, transform);
  cloud_inout = tmp;
}

/** Transform a point cloud in a given target TF frame using the given transfomer.
 * @param target_frame the target TF frame the point cloud should be transformed to
 * @param cloud_in input point cloud
 * @param cloud_out output point cloud
 * @param transformer TF transformer
 * @exception tf::TransformException if transform retrieval fails
 */
template <typename PointT>
void transform_pointcloud(const std::string &target_frame, 
			  const pcl::PointCloud<PointT> &cloud_in, 
			  pcl::PointCloud<PointT> &cloud_out, 
			  const tf::Transformer &transformer)
{
  if (cloud_in.header.frame_id == target_frame) {
    cloud_out = cloud_in;
    return;
  }

  fawkes::Time source_time;
  pcl_utils::get_time(cloud_in, source_time);
  tf::StampedTransform transform;
  transformer.lookup_transform(target_frame, cloud_in.header.frame_id,
			       source_time, transform);

  transform_pointcloud(cloud_in, cloud_out, transform);
  cloud_out.header.frame_id = target_frame;
}

/** Transform a point cloud in a given target TF frame using the given transfomer.
 * @param target_frame the target TF frame the point cloud should be transformed to
 * @param cloud_inout input and output point cloud
 * @param transformer TF transformer
 * @exception tf::TransformException if transform retrieval fails
 */
template <typename PointT>
void transform_pointcloud(const std::string &target_frame, 
			  pcl::PointCloud<PointT> &cloud_inout, 
			  const tf::Transformer &transformer)
{
  pcl::PointCloud<PointT> tmp;
  transform_pointcloud(target_frame, cloud_inout, tmp, transformer);
  cloud_inout = tmp;
}


/** Transform a point cloud in a given target TF frame using the given transfomer.
 * @param target_frame the target TF frame the point cloud should be transformed to
 * @param cloud_in input point cloud
 * @param cloud_out output point cloud
 * @param transformer TF transformer
 * @exception tf::TransformException if transform retrieval fails
 */
template <typename PointT> 
void transform_pointcloud(const std::string &target_frame, const Time &target_time,
			  const std::string &fixed_frame,
			  const pcl::PointCloud<PointT> &cloud_in, 
			  pcl::PointCloud<PointT> &cloud_out, 
			  const tf::Transformer &transformer)
{
  if (cloud_in.header.frame_id == target_frame) {
    cloud_out = cloud_in;
    return;
  }

  fawkes::Time source_time;
  pcl_utils::get_time(cloud_in, source_time);

  tf::StampedTransform transform;
  transformer.lookup_transform(target_frame, target_time,
			       cloud_in.header.frame_id, source_time,
			       fixed_frame, transform);

  transform_pointcloud(cloud_in, cloud_out, transform);
  cloud_out.header.frame_id = target_frame;

  pcl_utils::set_time(cloud_out, target_time);
}

/** Transform a point cloud in a given target TF frame using the given transfomer.
 * @param target_frame the target TF frame the point cloud should be transformed to
 * @param cloud_inout input and output point cloud
 * @param transformer TF transformer
 * @exception tf::TransformException if transform retrieval fails
 */
template <typename PointT> 
void transform_pointcloud(const std::string &target_frame, const Time &target_time,
			  const std::string &fixed_frame,
			  pcl::PointCloud<PointT> &cloud_inout, 
			  const tf::Transformer &transformer)
{
  pcl::PointCloud<PointT> tmp;
  transform_pointcloud(target_frame, target_time, fixed_frame,
		       cloud_inout, tmp, transformer);
  cloud_inout = tmp;
}

} // end namespace pcl_utils
} // end namespace fawkes

#endif
