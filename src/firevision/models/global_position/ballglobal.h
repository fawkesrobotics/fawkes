
/***************************************************************************
 *  ballglobal.h - A simple implementation of the global position model for
 *                 a ball
 *
 *  Generated: Fri Jun 03 22:56:22 2005
 *  Copyright  2005  Hu Yuxiao <Yuxiao.Hu@rwth-aachen.de>
 *                   Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */
                                                                                
#ifndef __FIREVISION_MODELS_GLOBAL_POSITION_BALL_H_
#define __FIREVISION_MODELS_GLOBAL_POSITION_BALL_H_

#include <models/global_position/globalpositionmodel.h>

class RelativePositionModel;

class BallGlobal : public GlobalPositionModel
{
 public:
  BallGlobal(RelativePositionModel* model);
  virtual void	setRobotPosition(float x, float y, float ori);
  virtual void  setPositionInImage(unsigned int x, unsigned int y);
  virtual float	getX(void) const;
  virtual float	getY(void) const;

  virtual void  calc();

  virtual bool  isPosValid() const;

 private:
  RelativePositionModel *m_pRelaModel;
  float	                 m_fPosX;
  float			 m_fPosY;
  float			 m_fPhi;

};

#endif
