
/***************************************************************************
 *  omni_global.h - Global position model that operating on a MirrorModel
 *
 *  Generated: Thu Mar 23 22:30:28 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_MODELS_GLOBAL_POSITION_OMNI_GLOBAL_H_
#define __FIREVISION_MODELS_GLOBAL_POSITION_OMNI_GLOBAL_H_

#include <models/global_position/globalpositionmodel.h>

class MirrorModel;

class OmniGlobal : public GlobalPositionModel
{
 public:
  // constructor
  OmniGlobal(MirrorModel *mirror_model);

  virtual void		setRobotPosition(float x, float y, float ori);
  virtual void          setPositionInImage(unsigned int x, unsigned int y);

  virtual float		getX() const;
  virtual float		getY() const;

  virtual void          calc();

  virtual bool          isPosValid() const;

private:
  float	                pose_x;
  float			pose_y;
  float			pose_ori;

  float                 ball_x;
  float                 ball_y;

  MirrorModel          *mirror_model;

  unsigned int          image_x;
  unsigned int          image_y;

};

#endif

