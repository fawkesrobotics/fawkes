/***************************************************************************
 *  relvelo.h - A simple velocity model using the relative coordinates and
 *              robot velocity
 *
 *  Generated: Tue Oct 04 15:49:23 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/
                                                                                
#ifndef __FIREVISION_MODELS_VELOCITY_RELATIVE_H_
#define __FIREVISION_MODELS_VELOCITY_RELATIVE_H_

#include <models/velocity/velocitymodel.h>
#include <models/relative_position/relativepositionmodel.h>

#include <fvutils/base/types.h>

// include <utils/kalman_filter/ckalman_filter_2dim.h>
#include <list>

/** Position/time tuple. */
typedef struct {
  float   x;	/**< x pos */
  float   y;	/**< y pos */
  timeval t;	/**< time */
} vel_postime_t;

/** Velocity/time tuple. */
typedef struct {
  float   vx;	/**< vx in m/s */
  float   vy;	/**< vy in m/s */
  timeval t;	/**< time */
} vel_veltime_t;

class VelocityFromRelative : public VelocityModel
{
 public:
  VelocityFromRelative(RelativePositionModel* model, unsigned int max_history_length, unsigned int calc_interval);
  virtual ~VelocityFromRelative();

  virtual const char * getName() const;

  virtual void	setRobotPosition(float x, float y, float ori, timeval t);
  virtual void  setRobotVelocity(float vel_x, float vel_y, timeval t);
  virtual void  setPanTilt(float pan, float tilt);
  virtual void  setTime(timeval t);
  virtual void  setTimeNow();
  virtual void  getTime(long int *sec, long int *usec);

  virtual void  getVelocity(float *vel_x, float *vel_y);

  virtual float getVelocityX();
  virtual float getVelocityY();

  virtual void  calc();
  virtual void  reset();

  virtual coordsys_type_t getCoordinateSystem();

 private:
  RelativePositionModel *relative_pos_model;

  float                  robot_rel_vel_x;
  float                  robot_rel_vel_y;
  timeval                robot_rel_vel_t;
  timeval                vel_last_time;

  timeval                now;
  std::list<vel_postime_t *>  ball_history;
  std::list<vel_postime_t *>::iterator bh_it;

  int                    diff_sec;
  int                    diff_usec;

  float                  f_diff_sec;

  unsigned int           max_history_length;
  unsigned int           calc_interval;

  float                  cur_ball_x;
  float                  cur_ball_y;
  float                  cur_ball_dist;

  // for projection
  bool                   last_available;
  timeval                last_time;
  float                  last_x;
  float                  last_y;
  float                  proj_x;
  float                  proj_y;
  float                  last_proj_error_x;
  float                  last_proj_error_y;
  float                  proj_time_diff_sec;

  float                  diff_x;
  float                  diff_y;

  float                  velocity_x;
  float                  velocity_y;

  float                  avg_vx_sum;
  float                  avg_vy_sum;
  float                  avg_vx;
  float                  avg_vy;
  unsigned int           avg_vx_num;
  unsigned int           avg_vy_num;
  float                  rx;
  float                  ry;
  float                  age_factor;

  /*
  bool                   kalman_enabled;
  float                  var_proc_x;
  float                  var_proc_y;
  float                  var_meas_x;
  float                  var_meas_y;
  kalmanFilter2Dim      *kalman_filter;

  void                  applyKalmanFilter();
  */

};

#endif
