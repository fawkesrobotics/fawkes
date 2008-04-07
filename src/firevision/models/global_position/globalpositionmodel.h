
/***************************************************************************
 *  globalpositionmodel.h - Abstract class defining a position model for
 *                          calculation of global position
 *
 *  Generated: Tue May 31 13:43:22 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FIREVISION_GLOBALPOSITIONMODEL_H_
#define __FIREVISION_GLOBALPOSITIONMODEL_H_

class GlobalPositionModel
{

 public:
  virtual ~GlobalPositionModel();

  virtual void		setRobotPosition(float x, float y, float ori)	    = 0;
  virtual void          setPositionInImage(unsigned int x, unsigned int y)  = 0;

  virtual float		getX() const					    = 0;
  virtual float		getY() const					    = 0;

  virtual void          calc()                                              = 0;

  virtual bool          isPosValid() const                                  = 0;

};

#endif

