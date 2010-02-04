
/***************************************************************************
 *  classifier.h - Abstract class defining a (color) classifier
 *
 *  Created: Tue May 03 19:50:02 2005
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

#ifndef __FIREVISION_CLASSIFIERS_CLASSIFIER_H_
#define __FIREVISION_CLASSIFIERS_CLASSIFIER_H_

#include <fvutils/base/roi.h>
#include <list>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

typedef std::list<ROI> ROIList;

class Classifier
{

 public:
  Classifier(const char *name);
  virtual ~Classifier();

  virtual void set_src_buffer(unsigned char *yuv422_planar,
			      unsigned int width, unsigned int height);
  virtual const char *  name() const;

  virtual ROIList * classify()                      = 0;

 protected:
  /** Source buffer, encoded as YUV422_PLANAR */
  unsigned char *_src;
  /** Width in pixels of _src buffer */
  unsigned int   _width;
  /** Height in pixels of _src buffer */
  unsigned int   _height;

 private:
  char *__name;

};

} // end namespace firevision

#endif
