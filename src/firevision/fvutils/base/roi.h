
/***************************************************************************
 *  roi.h - Header for Region Of Interest (ROI) representation
 *
 *  Generated: Tue Mai 03 19:46:44 2005
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

#ifndef __FIREVISION_FVUTILS_ROI_H_
#define __FIREVISION_FVUTILS_ROI_H_

#include <fvutils/base/types.h>

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
  H_BALL          = 0,	/**< ball */
  H_BACKGROUND    = 1,	/**< background */
  H_ROBOT         = 2,	/**< robot */
  H_FIELD         = 3,	/**< field */
  H_GOAL_YELLOW   = 4,	/**< yellow goal */
  H_GOAL_BLUE     = 5,	/**< blue goal */
  H_LINE          = 6,	/**< line */
  H_UNKNOWN       = 7,	/**< unknown */
  H_SIZE          = 8	/**< size of enum */
} hint_t;


class ROI {
 public:

  ROI();

  void         setStart(point_t p);
  void         setStart(unsigned int x, unsigned int y);

  void         setWidth(unsigned int width);
  unsigned int getWidth() const;

  void         setHeight(unsigned int height);
  unsigned int getHeight() const;

  void         setImageWidth(unsigned int image_width);
  unsigned int getImageWidth() const;

  void         setImageHeight(unsigned int image_height);
  unsigned int getImageHeight() const;

  void         setLineStep(unsigned int step);
  unsigned int getLineStep() const;

  void         setPixelStep(unsigned int step);
  unsigned int getPixelStep() const;

  hint_t       getHint() const;
  void         setHint(hint_t hint);

  bool         contains(unsigned int x, unsigned int y);

  bool         neighbours(unsigned int x, unsigned int y, unsigned int margin) const;
  bool         neighbours(ROI *roi, unsigned int margin) const;

  void         extend(unsigned int x, unsigned int y);
  ROI&         operator+=(ROI &roi);
  void         grow(unsigned int margin);


  bool         operator<(const ROI &roi);
  bool         operator>(ROI &roi);
  bool         operator==(ROI &roi);
  bool         operator!=(ROI &roi);
  ROI&         operator=(ROI &roi);

  unsigned int getNumHintPoints();


  unsigned char*  getROIBufferStart(unsigned char *buffer) const;

  static ROI * fullImage(unsigned int width, unsigned int height);


 public: // Public for quick access
  /** ROI start */
  point_t      start;
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
  hint_t       hint;

  /** Minimum estimate of points in ROI that are attributed to the ROI hint */
  unsigned int num_hint_points;

 private:
  static ROI  *roi_full_image;

};

#endif
