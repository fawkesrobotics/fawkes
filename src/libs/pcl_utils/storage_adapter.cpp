
/***************************************************************************
 *  storage_adapter.cpp - Utility to store differently typed point clouds in
 *                        common container.
 *
 *  Created: Fri Nov 30 16:46:13 2012
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

#include <pcl_utils/storage_adapter.h>

namespace fawkes {
  namespace pcl_utils {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** @class StorageAdapter <pcl_utils/storage_adapter.h>
 * Adapter base class.
 * The adapter base class is required to store point clouds of
 * arbitrary point types.
 * @author Tim Niemueller
 *
 * @fn bool StorageAdapter::is_pointtype() const
 * Check if storage adapter is for specified point type.
 * @return true if storage adapter is for specified point type, false otherwise
 *
 * @fn template <PointT> PointCloudStorageAdapter<PointT> * StorageAdapter::as_pointtype() const
 * Transform to specific PointCloudStorageAdapter.
 * @return transformed PointCloudStorageAdapter
 * @exception Exception thrown if storage adapter is not of requested point type or does not exist
 * 
 * @fn StorageAdapter * StorageAdapter::clone() const
 * Clone this storage adapter.
 * @return cloned copy
 *
 * @fn const char * StorageAdapter::get_typename()
 * Get typename of storage adapter.
 * @return type name
 *
 * @fn size_t StorageAdapter::point_size() const
 * Get size of a point.
 * @return size in bytes of a single point
 *
 * @fn unsigned int StorageAdapter::width() const
 * Get width of point cloud.
 * @return width of point cloud
 *
 * @fn unsigned int StorageAdapter::height() const
 * Get height of point cloud.
 * @return height of point cloud
 *
 * @fn size_t StorageAdapter::num_points() const
 * Get numer of points in point cloud.
 * @return number of points
 *
 * @fn void * StorageAdapter::data_ptr() const
 * Get pointer on data.
 * @return pointer on data
 *
 * @fn  void StorageAdapter::get_time(fawkes::Time &time) const
 * Get last capture time.
 * @param time upon return contains last capture time
 *
 * @fn std::string & StorageAdapter::frame_id() const
 * Get frame ID of point cloud.
 * @return Frame ID of point cloud.
 *
 * @fn void StorageAdapter::transform(const std::string &target_frame, const tf::Transformer &transformer)
 * Transform point cloud.
 * @param target_frame frame to transform to
 * @param transformer transformer to get transform from
 *
 * @fn void StorageAdapter::transform(const std::string &target_frame, const Time &target_time, const std::string &fixed_frame, const tf::Transformer &transformer)
 * Transform point cloud.
 * @param target_frame frame to transform to
 * @param target_time time for which to transform
 * @param fixed_frame frame fixed over time
 * @param transformer transformer to get transform from
 */


/** @class PointCloudStorageAdapter <pcl_utils/storage_adapter.h>
 * Adapter class for PCL point types.
 * The adapter class is required to store point clouds of arbitrary point types.
 * @author Tim Niemueller
 */

} // end namespace pcl_utils
} // end namespace fawkes
