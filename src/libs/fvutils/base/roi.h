
/***************************************************************************
 *  roi.h - Header for Region Of Interest (ROI) representation
 *
 *  Generated: Tue Mai 03 19:46:44 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_ROI_H_
#define __FIREVISION_FVUTILS_ROI_H_

#include <fvutils/base/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/* The values of this enum-type have to be indexed subsequently,
   beginning with 0. The last value has to be "H_SIZE = ...".
   You may add further values at the end (but before H_SIZE!)
   just continue the indexing properly.
   NOTE: The indexing must be in correct order wrt the histograms
   used in "genlut"
   Do NOT change the order as this may invalidate already created
   color maps.
 */
/** Hint about object. */
typedef enum {
  H_BALL          = 0,  /**< ball */
  H_BACKGROUND    = 1,  /**< background */
  H_ROBOT         = 2,  /**< robot */
  H_FIELD         = 3,  /**< field */
  H_GOAL_YELLOW   = 4,  /**< yellow goal */
  H_GOAL_BLUE     = 5,  /**< blue goal */
  H_LINE          = 6,  /**< line */
  H_UNKNOWN       = 7,  /**< unknown */
  H_ROBOT_OPP     = 8,  /**< opponents robot */
  H_SIZE                /**< size of enum (Has to be the last entry) */
} hint_t;


class ROI {
 public:

  ROI();
  ROI(const ROI &roi);
  ROI(const ROI *roi);
  ROI(unsigned int start_x, unsigned int start_y,
      unsigned int width, unsigned int height,
      unsigned int image_width, unsigned int image_height);

  void         set_start(fawkes::upoint_t p);
  void         set_start(unsigned int x, unsigned int y);

  void         set_width(unsigned int width);
  unsigned int get_width() const;

  void         set_height(unsigned int height);
  unsigned int get_height() const;

  void         set_image_width(unsigned int image_width);
  unsigned int get_image_width() const;

  void         set_image_height(unsigned int image_height);
  unsigned int get_image_height() const;

  void         set_line_step(unsigned int step);
  unsigned int get_line_step() const;

  void         set_pixel_step(unsigned int step);
  unsigned int get_pixel_step() const;

  unsigned int get_hint() const;
  void         set_hint(unsigned int);

  bool         contains(unsigned int x, unsigned int y);
  ROI          intersect(ROI const &roi) const;

  bool         neighbours(unsigned int x, unsigned int y, unsigned int margin) const;
  bool         neighbours(ROI *roi, unsigned int margin) const;

  void         extend(unsigned int x, unsigned int y);
  ROI&         operator+=(ROI &roi);
  void         grow(unsigned int margin);


  bool         operator<(const ROI &roi) const;
  bool         operator>(const ROI &roi) const;
  bool         operator==(const ROI &roi) const;
  bool         operator!=(const ROI &roi) const;
  ROI&         operator=(const ROI &roi);

  unsigned int get_num_hint_points() const;


  unsigned char*  get_roi_buffer_start(unsigned char *buffer) const;

  static ROI * full_image(unsigned int width, unsigned int height);


 public: // Public for quick access
  /** ROI start */
  fawkes::upoint_t start;
  /** ROI width */
  unsigned int width;
  /** ROI height */
  unsigned int height;
  /** width of image that contains this ROI */
  unsigned int image_width;
  /** height of image that contains this ROI */
  unsigned int image_height;
  /** line step */
  unsigned int line_step;
  /** pixel step */
  unsigned int pixel_step;
  /** ROI hint */
  unsigned int hint;

  /** ROI primary color */
  color_t      color;

  /** Minimum estimate of points in ROI that are attributed to the ROI hint */
  unsigned int num_hint_points;

 private:
  static ROI  *roi_full_image;

};

} // end namespace firevision

#endif
