
/***************************************************************************
 *  classifier.h - Abstract class defining a (color) classifier
 *
 *  Created: Tue May 03 19:50:02 2005
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

#ifndef __FIREVISION_CLASSIFIERS_CLASSIFIER_H_
#define __FIREVISION_CLASSIFIERS_CLASSIFIER_H_

#include <fvutils/base/roi.h>
#include <list>

class Classifier
{

 public:
  Classifier(const char *name);
  virtual ~Classifier();

  virtual void set_src_buffer(unsigned char *yuv422_planar,
			      unsigned int width, unsigned int height);
  virtual const char *  name() const;

  virtual std::list< ROI > * classify()                      = 0;

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

#endif
