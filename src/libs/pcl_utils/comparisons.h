
/***************************************************************************
 *  comparisons.h - PCL utilities: additional comparison functors
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

#ifndef __LIBS_PCL_UTILS_COMPARISONS_H_
#define __LIBS_PCL_UTILS_COMPARISONS_H_

#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

namespace fawkes {
  namespace pcl_utils {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif


/** Check if point is inside or outside a given polygon.
 * This comparison determines if a given point is inside or outside a
 * given polygon. A flag can be set to have an inside or outside
 * check. The class uses pcl::isPointIn2DPolygon() to determine if the
 * point is inside the polygon. Not that we assume planar data, for
 * example points projected into a segmented plane.
 * @author Tim Niemueller
 */
template <typename PointT>
class PolygonComparison : public pcl::ComparisonBase<PointT>
{
  using pcl::ComparisonBase<PointT>::capable_;
 public:
  /// Shared pointer.
  typedef boost::shared_ptr<PolygonComparison<PointT> > Ptr;
  /// Constant shared pointer.
  typedef boost::shared_ptr<const PolygonComparison<PointT> > ConstPtr;
 
  /** Constructor.
   * @param polygon polygon to compare against, it must have at least three points
   * @param inside if true filter points inside the polygon, false for outside
   */
  PolygonComparison(const pcl::PointCloud<PointT> &polygon, bool inside = true)
    : inside_(inside), polygon_(polygon)
  {
    capable_ = (polygon.size() >= 3);
  }
  /** Virtual empty destructor. */
  virtual ~PolygonComparison() {}
 
  /** Evaluate for given pixel.
   * @param point point to compare
   * @return true if the point is inside/outside (depending on
   * constructor parameter) the polygon, false otherwise
   */
  virtual bool evaluate(const PointT &point) const
  {
    if (inside_)
      return   pcl::isPointIn2DPolygon(point, polygon_);
    else
      return ! pcl::isPointIn2DPolygon(point, polygon_);
  }
 
 protected:
  /// Flag to determine whether to do inside or outside check
  bool inside_;
  /// The polygon to check against
  const pcl::PointCloud<PointT> &polygon_;
 
 private:
  PolygonComparison() {} // not allowed
};


/** Compare points' distance to a plane.
 * This comparison calculates the distance to a given plane and makes
 * a decision based on constructor flag and threshold.
 * @author Tim Niemueller
 */
template <typename PointT>
class PlaneDistanceComparison : public pcl::ComparisonBase<PointT>
{
  using pcl::ComparisonBase<PointT>::capable_;
 public:
  /// Shared pointer.
  typedef boost::shared_ptr<PlaneDistanceComparison<PointT> > Ptr;
  /// Constant shared pointer.
  typedef boost::shared_ptr<const PlaneDistanceComparison<PointT> > ConstPtr;
 
  /** Constructor.
   * @param coeff planar model coefficients
   * @param op comparison operation
   * @param compare_val value to compare against
   */
  PlaneDistanceComparison(pcl::ModelCoefficients::ConstPtr coeff,
                          pcl::ComparisonOps::CompareOp op = pcl::ComparisonOps::GT,
                          float compare_val = 0.)
    : coeff_(coeff), op_(op), compare_val_(compare_val)
  {
    capable_ = (coeff_->values.size() == 4);
  }
  /** Virtual empty destructor. */
  virtual ~PlaneDistanceComparison() {}
 
  /** Evaluate for given pixel.
   * @param point point to compare
   * @return true if the setup operation using the compare value evaluates to true, false otherwise
   */
  virtual bool evaluate(const PointT &point) const
  {
    float val = (coeff_->values[0] * point.x + coeff_->values[1] * point.y +
                 coeff_->values[2] * point.z + coeff_->values[3]) /
      sqrtf(coeff_->values[0] * coeff_->values[0] +
            coeff_->values[1] * coeff_->values[1] +
            coeff_->values[2] * coeff_->values[2]);

    //printf("%f > %f?: %d\n", val, compare_val_, (val > compare_val_));

    if (op_ == pcl::ComparisonOps::GT) {
      return val >  compare_val_;
    } else if (op_ == pcl::ComparisonOps::GE) {
      return val >= compare_val_;
    } else if (op_ == pcl::ComparisonOps::LT) {
      return val <  compare_val_;
    } else if (op_ == pcl::ComparisonOps::LE) {
      return val <= compare_val_;
    } else {
      return val == compare_val_;
    }
  }
 
 protected:
  /// Planar model coefficients
  pcl::ModelCoefficients::ConstPtr coeff_;
  /// Comparison operation
  pcl::ComparisonOps::CompareOp op_;
  /// Value to compare against
  float compare_val_;

 private:
  PlaneDistanceComparison() {} // not allowed
};

} // end namespace pclutils
} // end namespace fawkes

#endif
