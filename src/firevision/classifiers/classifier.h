
/***************************************************************************
 *  classifier.h - Abstract class defining a (color) classifier
 *
 *  Generated: Tue May 03 19:50:02 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_COLORCLASSIFIER_H_
#define __FIREVISION_COLORCLASSIFIER_H_

#include <list>
#include <fvutils/types.h>
#include <fvutils/roi.h>

class Classifier
{

 public:
  virtual ~Classifier();

  virtual void setSrcBuffer(unsigned char *buf)     = 0;

  virtual const char *  getName() const             = 0;

  virtual std::list< ROI > * classify()             = 0;

};

#endif
