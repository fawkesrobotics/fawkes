
/***************************************************************************
 *  scanlinemodel.h - Abstract class defining a scanline model
 *
 *  Generated: Tue May 03 19:50:02 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_MODELS_SCANLINES_SCANLINEMODEL_H_
#define __FIREVISION_MODELS_SCANLINES_SCANLINEMODEL_H_


/* IMPORTANT IMPLEMENTATION NOTE:
 * Do _not_ split this into a .h and .cpp file. This interface is used in fvutils.
 * The only way to allows this circular dependency is to have this as a plain
 * header that does not require any linking. Thus you may not split the file.
 */

#include <fvutils/base/types.h>
#include <string>

/** @class ScanlineModel <models/scanlines/scanlinemodel.h>
 * Scanline model interface.
 * This interface defines the API for the scanline model. A scanline model
 * determines a specific set of points in the image that should be used for image
 * evaluation if using all the pixels of an image would take too long.
 * This is one of the major optimizations throughout FireVision to ensure high
 * speed image processing.
 *
 * @author Tim Niemueller
 */

class ScanlineModel
{

 public:
  /** Virtual empty destructor. */
  virtual ~ScanlineModel() {}

  /** Get the current coordinate.
   * @return current point in image that is shall be processed.
   */
  virtual point_t       operator*()                                    = 0;

  /** Get pointer to current point.
   * @return pointer to current point
   * @see operator*()
   */
  virtual point_t*      operator->()                                   = 0;

  /** Postfix ++ operator.
   * Advances to the next point and returns the new point.
   * @return pointer to new point
   */
  virtual point_t*      operator++()                                   = 0;

  /** Prefix ++ operator.
   * Advances to the next point but returns the old point.
   */
  virtual point_t*      operator++(int)                                = 0;

  /** Check if all desired points have been processed.
   * @return true if all pixels that the model defines have been iterated.
   */
  virtual bool          finished()                                     = 0;

  /** Reset model.
   * Resets the set of processed points.
   */
  virtual void          reset()                                        = 0;

  /** Get name of scanline model.
   * @return name of scanline model.
   */
  virtual const char *  getName()                                      = 0;


  /** Get margin around points.
   * Models that do not use margins shall return zero.
   * It shall be guaranteed that in this margin
   * region around a point there is no other point that has been or will be
   * returned in a full iteration.
   * @return margin around a point.
   */
  virtual unsigned int  getMargin()                                    = 0;

  /** Set the robot's pose.
   * @param x robot's x coordinate on field in meters
   * @param y robot's y coordinate on field in meters
   * @param ori robot's orientation. Looking towards the opponent goal is zero
   * rad, with positive values pointing to the right, negative to the left.
   */
  virtual void          setRobotPose(float x, float y, float ori)      = 0;

  /** Set camera's pan/tilt values.
   * @param pan camera's current pan
   * @param tilt camera's current tilt
   */
  virtual void          setPanTilt(float pan, float tilt)              = 0;

};

#endif
