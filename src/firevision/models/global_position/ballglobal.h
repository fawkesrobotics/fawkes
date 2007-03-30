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
