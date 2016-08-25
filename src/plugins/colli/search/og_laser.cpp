
/***************************************************************************
 *  og_laser.cpp - Occupancy grid for colli's A* search
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013-2014  Bahram Maleki-Fard
 *             2014  Tobias Neumann
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "og_laser.h"
#include "obstacle_map.h"

#include "../utils/rob/roboshape_colli.h"

#include <utils/time/clock.h>
#include <utils/math/angle.h>
#include <utils/math/coord.h>

#include <logging/logger.h>
#include <config/config.h>

#include <interfaces/Laser360Interface.h>

#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LaserOccupancyGrid <plugins/colli/search/og_laser.h>
 *  This OccGrid is derived by the Occupancy Grid originally from Andreas Strack,
 *    but modified for speed purposes.
 */

/** Constructor.
 * @param laser The Laser object
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 * @param listener The tf::Transformer
 * @param width The width of the grid (in m)
 * @param height The height of the grid (in m)
 * @param cell_width The width of a cell (in cm)
 * @param cell_height The height of a cell (in cm)
 */
LaserOccupancyGrid::LaserOccupancyGrid( Laser360Interface * laser, Logger* logger, Configuration* config, tf::Transformer* listener,
                                        int width, int height, int cell_width, int cell_height)
 : OccupancyGrid( width, height, cell_width, cell_height ),
   tf_listener_(listener ),
   logger_( logger ),
   if_laser_( laser )
{
  logger->log_debug("LaserOccupancyGrid", "(Constructor): Entering");

  //read config
  std::string cfg_prefix = "/plugins/colli/";
  obstacle_distance_     = config->get_float((cfg_prefix + "laser_occupancy_grid/distance_account").c_str());
  initial_history_size_  = 3*config->get_int((cfg_prefix + "laser_occupancy_grid/history/initial_size").c_str());
  max_history_length_    = config->get_float((cfg_prefix + "laser_occupancy_grid/history/max_length").c_str());
  min_history_length_    = config->get_float((cfg_prefix + "laser_occupancy_grid/history/min_length").c_str());
  min_laser_length_      = config->get_float((cfg_prefix + "laser/min_reading_length").c_str());
  cfg_write_spam_debug_  = config->get_bool((cfg_prefix + "write_spam_debug").c_str());

  cfg_emergency_stop_beams_used_ = config->get_float((cfg_prefix + "emergency_stopping/beams_used").c_str());

  cfg_delete_invisible_old_obstacles_           = config->get_bool((cfg_prefix + "laser_occupancy_grid/history/delete_invisible_old_obstacles/enable").c_str());
  cfg_delete_invisible_old_obstacles_angle_min_ = config->get_int((cfg_prefix + "laser_occupancy_grid/history/delete_invisible_old_obstacles/angle_min").c_str());
  cfg_delete_invisible_old_obstacles_angle_max_ = config->get_int((cfg_prefix + "laser_occupancy_grid/history/delete_invisible_old_obstacles/angle_max").c_str());
  if ( cfg_delete_invisible_old_obstacles_angle_min_ >= 360 ) {
    logger_->log_warn("LaserOccupancyGrid", "Min angle out of bounce, use 0");
    cfg_delete_invisible_old_obstacles_angle_min_ = 0;
  }
  if ( cfg_delete_invisible_old_obstacles_angle_min_ >= 360 ) {
    logger_->log_warn("LaserOccupancyGrid", "Max angle out of bounce, use 360");
    cfg_delete_invisible_old_obstacles_angle_min_ = 360;
  }

  if (cfg_delete_invisible_old_obstacles_angle_max_ > cfg_delete_invisible_old_obstacles_angle_min_) {
    angle_range_ = deg2rad((unsigned int)abs(cfg_delete_invisible_old_obstacles_angle_max_ - cfg_delete_invisible_old_obstacles_angle_min_));
  } else {
    angle_range_ = deg2rad((360 - cfg_delete_invisible_old_obstacles_angle_min_) + cfg_delete_invisible_old_obstacles_angle_max_);
  }
  angle_min_ = deg2rad( cfg_delete_invisible_old_obstacles_angle_min_ );


  reference_frame_ = config->get_string((cfg_prefix + "frame/odometry").c_str());
  laser_frame_     = config->get_string((cfg_prefix + "frame/laser").c_str());       //TODO change to base_link => search in base_link instead base_laser

  cfg_obstacle_inc_          = config->get_bool((cfg_prefix + "obstacle_increasement").c_str());
  cfg_force_elipse_obstacle_ = config->get_bool((cfg_prefix + "laser_occupancy_grid/force_ellipse_obstacle").c_str());

  if_buffer_size_ = config->get_int((cfg_prefix + "laser_occupancy_grid/buffer_size").c_str());
  if_buffer_size_ = std::max(if_buffer_size_, 1); //needs to be >= 1, because the data is always wrote into the buffer (instead of read())

  cell_costs_.occ  = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/occupied").c_str());
  cell_costs_.near = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/near").c_str());
  cell_costs_.mid  = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/mid").c_str());
  cell_costs_.far  = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/far").c_str());
  cell_costs_.free = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/free").c_str());

  if_buffer_filled_.resize(if_buffer_size_);
  std::fill(if_buffer_filled_.begin(), if_buffer_filled_.end(), false);

  if_laser_->resize_buffers( if_buffer_size_ );

  robo_shape_ = new RoboShapeColli( (cfg_prefix + "roboshape/").c_str(), logger, config );
  old_readings_.clear();
  init_grid();

  logger->log_debug("LaserOccupancyGrid", "Generating obstacle map");
  bool obstacle_shape = robo_shape_->is_angular_robot() && ! cfg_force_elipse_obstacle_;
  obstacle_map = new ColliObstacleMap(cell_costs_, obstacle_shape);
  logger->log_debug("LaserOccupancyGrid", "Generating obstacle map done");

  laser_pos_ = point_t(0,0);

  // calculate laser offset from robot center
  offset_base_.x=0;
  offset_base_.y=0;
  offset_laser_.x = robo_shape_->get_complete_width_x()/2.f - robo_shape_->get_robot_length_for_deg(0);
  offset_laser_.y = robo_shape_->get_complete_width_y()/2.f - robo_shape_->get_robot_length_for_deg(90);
  logger->log_debug("LaserOccupancyGrid", "Laser (x,y) offset from robo-center is (%f, %f)",
                    offset_laser_.x, offset_laser_.y);


  logger->log_debug("LaserOccupancyGrid", "(Constructor): Exiting");
}

/** Descturctor. */
LaserOccupancyGrid::~LaserOccupancyGrid()
{
  delete robo_shape_;
}

/** Reset all old readings and forget about the world state! */
void
LaserOccupancyGrid::reset_old()
{
  old_readings_.clear();
  old_readings_.reserve( initial_history_size_ );
}

/**
 * Gets data from laser (does! read it) and transforms them into the reference-frame (odom)
 */
void
LaserOccupancyGrid::update_laser()
{
  //check for free pos in buffer
  int if_buffer_free_pos = -1;

  for (int i = 0; i < if_buffer_size_; ++i) {    //for all buffer possition
    if (if_buffer_filled_[i] == false) {         //if free (used == false)
      if_buffer_free_pos = i;                     //use this buffer
      //stop loop
    }
  }
  //write BB date into buffer (instead of read())
  if ( if_buffer_free_pos < 0 ) {                 //if there is no free buffer
	  //logger_->log_error("LaserOccupancyGrid", "if_laser buffer is full empty oldest");

                                                  //search for the oldest buffer and uses this
    double if_buffer_oldest_time = Clock::instance()->now().in_sec() + 1000;
    int if_buffer_oldest_pos = -1;

    for (int i = 0; i < if_buffer_size_; ++i) {
      if (if_laser_->buffer_timestamp( i ).in_sec() < if_buffer_oldest_time) {
        if_buffer_oldest_pos = i;
        if_buffer_oldest_time = if_laser_->buffer_timestamp( i ).in_sec();
      }
    }
    if_buffer_free_pos = if_buffer_oldest_pos;
  }

  if_laser_->copy_shared_to_buffer( if_buffer_free_pos );     //read new laser data
  if_buffer_filled_[ if_buffer_free_pos ] = true;            //set buffer used

  new_readings_.clear();
  new_readings_.reserve( if_laser_->maxlenof_distances() * if_buffer_size_ );
  //for all buffer: try to transform and save in grid
  for (int i = 0; i < if_buffer_size_; ++i) {
    if (if_buffer_filled_[i] == true) {      //if is filled

      if_laser_->read_from_buffer( i );         //read buffer
      if_buffer_filled_[i] = false;            //show buffer is not used
      //TODO just if there are new data
      const Time* laser_time  = if_laser_->timestamp();
      std::string laser_frame = if_laser_->frame();

      tf::StampedTransform transform;

      try {
        tf_listener_->lookup_transform(reference_frame_, laser_frame, laser_time, transform);

        tf::Vector3 pos_robot_tf = transform.getOrigin();
        cart_coord_2d_t pos_robot(pos_robot_tf.getX(), pos_robot_tf.getY());

        double angle_inc = M_PI * 2. / 360.;
        tf::Point p;
        //Save all Points in refernce Frame
        for (unsigned int i = 0; i < if_laser_->maxlenof_distances(); ++i) {
          if (if_laser_->distances(i) >= min_laser_length_) {
            //Save as polar coordinaten
            polar_coord_2d_t point_polar;
            point_polar.r   = if_laser_->distances(i);
            point_polar.phi = angle_inc * i;

            //Calculate as cartesien
            cart_coord_2d_t point_cart;
            polar2cart2d(point_polar.phi, point_polar.r, &point_cart.x, &point_cart.y);

            //transform into odom
            p.setValue(point_cart.x, point_cart.y, 0.);
            p = transform * p;

            LaserOccupancyGrid::LaserPoint point;
            point.coord     = cart_coord_2d_t( p.getX(), p.getY() );
            point.timestamp = Time(laser_time);

            new_readings_.push_back(point);

            if ( cfg_delete_invisible_old_obstacles_ ) {
              float angle_dist = angle_distance_signed( angle_min_, point_polar.phi );
              if ( angle_dist >= 0 && angle_dist <= angle_range_ ) {
                validate_old_laser_points(pos_robot, point.coord);
              }
            }
          }
        }
      } catch(Exception &e) {
        if_buffer_filled_[i] = true;            //show buffer still needs to be there
        if (cfg_write_spam_debug_) {
          logger_->log_debug("LaserOccupancyGrid", "Unable to transform %s to %s. Laser-data not used, will keeped in history.",
                              laser_frame.c_str(), reference_frame_.c_str());
        }
      }
    }
  }
}

/**
 * compare the given point with all old points to delete old-wrong-obstacles
 * @param pos_robot           the robot pose where the point to compare with where taken
 * @param pos_new_laser_point the position of the point to compare with
 */
void
LaserOccupancyGrid::validate_old_laser_points(cart_coord_2d_t pos_robot, cart_coord_2d_t pos_new_laser_point)
{
  std::vector< LaserPoint > old_readings_tmp;

  // vectors from robot to new and old laser-points
  cart_coord_2d_t v_new(pos_new_laser_point.x - pos_robot.x , pos_new_laser_point.y - pos_robot.y);
  cart_coord_2d_t v_old;

  // distances from robot to new and old laser-points (i.e. length of v_new and v_old)
  float d_new = sqrt(v_new.x*v_new.x + v_new.y*v_new.y);
  float d_old = 0.f;

  // angle between the two vectors v_new and v_old. Use to determine whether they
  // belong to the same laser-beam
  float angle = 0.f;

  static const float deg_unit = M_PI / 180.f; // 1 degree

  for ( std::vector< LaserPoint >::iterator it = old_readings_.begin();
      it != old_readings_.end(); ++it ) {

    v_old.x = (*it).coord.x - pos_robot.x;
    v_old.y = (*it).coord.y - pos_robot.y;

    // need to calculate distance here, needed for angle calculation
    d_old = sqrt(v_old.x*v_old.x + v_old.y*v_old.y);

    // we already have the distances, so already make the distance-check here
    if( d_new <= d_old + obstacle_distance_ ) {
      // in case both points belonged to the same laser-beam, p_old
      // would be in shadow of p_new => keep p_old anyway
      old_readings_tmp.push_back( *it );
      continue;
    }

    // angle a between to vectors v,w: cos(a) = dot(v,w) / (|v|*|w|)
    angle = acos( (v_old.x*v_new.x + v_old.y*v_new.y) / (d_new*d_old) );
    if( std::isnan(angle) || angle > deg_unit ) {
      // p_old is not the range of this laser-beam. Keep it.
      old_readings_tmp.push_back( *it );

      /* No "else" here. It would mean that p_old is in the range of the
       * same laser beam. And we already know that
       * "d_new > d_old + obstacle_distance_" => this laser beam can see
       * through p_old => discard p_old. In other words, do not add to
       * old_readings_tmp.
       */
     }
  }

  old_readings_.clear();
  old_readings_.reserve( old_readings_tmp.size() );

  for (unsigned int i = 0; i < old_readings_tmp.size(); ++i) {
    old_readings_.push_back( old_readings_tmp[i] );
  }
}

float
LaserOccupancyGrid::obstacle_in_path_distance( float vx, float vy )
{
  if_laser_->read();
  int angle = roundf( rad2deg( normalize_rad( atan2f(vy, vx) ) ) );

  float distance_min = 1000;

  int cfg_beams = cfg_emergency_stop_beams_used_;

  int beams_start = angle - int( cfg_beams / 2 );
  if ( beams_start < 0 )  { beams_start += 360; }

  int beams_end   = beams_start + cfg_beams;
  if ( beams_end >= 360 ) { beams_end -= 360; }

  for (int i = beams_start; i != beams_end; i = (i+1) % 360 ) {
    float dist = if_laser_->distances(i);
    if ( dist != 0 && std::isfinite(dist) ) {
      distance_min = std::min( distance_min, dist );
    }
  }

  return distance_min;
}

/** Put the laser readings in the occupancy grid
 *  Also, every reading gets a radius according to the relative direction
 *  of this reading to the robot.
 * @param midX is the current x position of the robot in the grid.
 * @param midY is the current y position of the robot in the grid.
 * @param inc is the current constant to increase the obstacles.
 * @param vx Translation x velocity of the motor
 * @param vy Translation y velocity of the motor
 * @return distance to next obstacle in pathdirection
 */
float
LaserOccupancyGrid::update_occ_grid( int midX, int midY, float inc, float vx, float vy )
{
  float vel = std::sqrt(vx*vx + vy*vy);

  float next_obstacle = obstacle_in_path_distance( vx, vy );

  laser_pos_.x = midX;
  laser_pos_.y = midY;

  for ( int y = 0; y < width_; ++y )
    for ( int x = 0; x < height_; ++x )
      occupancy_probs_[x][y] = cell_costs_.free;

  update_laser();

  tf::StampedTransform transform;

  try {
    tf_listener_->lookup_transform(laser_frame_, reference_frame_, Time(0,0), transform);

  } catch(Exception &e) {
    logger_->log_error("LaserOccupancyGrid", "Unable to transform %s to %s. Can't put obstacles into the grid",
        reference_frame_.c_str(), laser_frame_.c_str());
    return 0.;
  }

  integrate_old_readings( midX, midY, inc, vel, transform );
  integrate_new_readings( midX, midY, inc, vel, transform );

  return next_obstacle;
}

/**
 * Transforms all given points with the given transform
 * @param laserPoints vector of LaserPoint, that contains the points to transform
 * @param transform stamped transform, the transform to transform with
 * @return the transformed laserPoints
 */
std::vector< LaserOccupancyGrid::LaserPoint >*
LaserOccupancyGrid::transform_laser_points(std::vector< LaserOccupancyGrid::LaserPoint >& laserPoints, tf::StampedTransform& transform)
{
  int count_points = laserPoints.size();
  std::vector< LaserOccupancyGrid::LaserPoint >* laserPointsTransformed = new std::vector< LaserOccupancyGrid::LaserPoint >();
  laserPointsTransformed->reserve( count_points );

  tf::Point p;

  for (int i = 0; i < count_points; ++i) {
    p.setValue(laserPoints[i].coord.x, laserPoints[i].coord.y, 0.);
    p = transform * p;

    LaserOccupancyGrid::LaserPoint point;
    point.coord     = cart_coord_2d_struct( p.getX(), p.getY() );
    point.timestamp = laserPoints[i].timestamp;
    laserPointsTransformed->push_back( point );
  }

  return laserPointsTransformed;
}

/** Get the laser's position in the grid
 * @return point_t structure containing the laser's position in the grid
 */
point_t
LaserOccupancyGrid::get_laser_position()
{
  return laser_pos_;
}

/** Set the offset of base_link from laser.
 * @param x offset in x-direction (in meters)
 * @param y offset in y-direction (in meters)
 */
void
LaserOccupancyGrid::set_base_offset(float x, float y)
{
  offset_base_.x = (int)round( (offset_laser_.x + x)*100.f/cell_height_ ); // # in grid-cells
  offset_base_.y = (int)round( (offset_laser_.y + y)*100.f/cell_width_  );
}


/** Get cell costs.
 * @return struct that contains all the cost values for the occgrid cells
 */
colli_cell_cost_t
LaserOccupancyGrid::get_cell_costs() const
{
  return cell_costs_;
}

void
LaserOccupancyGrid::integrate_old_readings( int midX, int midY, float inc, float vel,
                                           tf::StampedTransform& transform )
{
  std::vector< LaserOccupancyGrid::LaserPoint > old_readings;
  old_readings.reserve( old_readings_.size() );
  std::vector< LaserOccupancyGrid::LaserPoint >* pointsTransformed = transform_laser_points(old_readings_, transform);

  float newpos_x, newpos_y;

  Clock* clock = Clock::instance();
  Time history = Time(clock) - Time(double(std::max( min_history_length_, max_history_length_)));

  // update all old readings
  for ( unsigned int i = 0; i < pointsTransformed->size(); ++i ) {

    if ( (*pointsTransformed)[i].timestamp.in_sec() >= history.in_sec() ) {

      newpos_x =  (*pointsTransformed)[i].coord.x;
      newpos_y =  (*pointsTransformed)[i].coord.y;

      //newpos_x =  old_readings_[i].coord.x + xref;
      //newpos_y =  old_readings_[i].coord.y + yref;

      //float angle_to_old_reading = atan2( newpos_y, newpos_x );
      //float sqr_distance_to_old_reading = sqr( newpos_x ) + sqr( newpos_y );

      //int number_of_old_reading = (int)rad2deg(
      //    normalize_rad(360.0/m_pLaser->GetNumberOfReadings() * angle_to_old_reading) );
      // This was RCSoftX, now ported to fawkes:
      //int number_of_old_reading = (int) (normalize_degree( ( 360.0/(m_pLaser->GetNumberOfReadings()) ) *
      //         rad2deg(angle_to_old_reading) ) );


      bool SollEintragen = true;

      // do not insert if current reading at that angle deviates more than 30cm from old reading
      // TODO. make those 30cm configurable
      //if ( sqr( m_pLaser->GetReadingLength( number_of_old_reading ) - 0.3 ) > sqr_distance_to_old_reading )
      //  SollEintragen = false;

      if ( SollEintragen == true ) {
        int posX = midX + (int)((newpos_x*100.f) / ((float)cell_height_ ));
        int posY = midY + (int)((newpos_y*100.f) / ((float)cell_width_ ));
        if( posX > 4 && posX < height_-5
         && posY > 4 && posY < width_-5 )
          {
          old_readings.push_back( old_readings_[i] );

          // 25 cm's in my opinion, that are here: 0.25*100/cell_width_
          //int size = (int)(((0.25f+inc)*100.f)/(float)cell_width_);
          float width = robo_shape_->get_complete_width_y();
          width = std::max( 4.f, ((width + inc)*100.f)/cell_width_ );
          float height = robo_shape_->get_complete_width_x();
          height = std::max( 4.f, ((height + inc)*100.f)/cell_height_ );
          integrate_obstacle( posX, posY, width, height );
        }
      }

    }
  }

  old_readings_.clear();
  old_readings_.reserve( old_readings.size() );

  // integrate the new calculated old readings
  for ( unsigned int i = 0; i < old_readings.size(); i++ )
    old_readings_.push_back( old_readings[i] );

  delete pointsTransformed;
}


void
LaserOccupancyGrid::integrate_new_readings( int midX, int midY, float inc, float vel,
                                           tf::StampedTransform& transform )
{
  std::vector< LaserOccupancyGrid::LaserPoint >* pointsTransformed = transform_laser_points(new_readings_, transform);

  int numberOfReadings = pointsTransformed->size();
  //TODO resize, reserve??

  int posX, posY;
  cart_coord_2d_t point;
  float oldp_x = 1000.f;
  float oldp_y = 1000.f;

  for ( int i = 0; i < numberOfReadings; i++ ) {
    point = (*pointsTransformed)[i].coord;

    if( sqrt(sqr(point.x) + sqr(point.y)) >= min_laser_length_
     && distance(point.x, point.y, oldp_x, oldp_y) >= obstacle_distance_)
      {
      oldp_x = point.x;
      oldp_y = point.y;
      posX = midX + (int)((point.x*100.f) / ((float)cell_height_ ));
      posY = midY + (int)((point.y*100.f) / ((float)cell_width_ ));

      if ( !( posX <= 5 || posX >= height_-6 || posY <= 5 || posY >= width_-6 ) ) {
        float width = 0.f;
        width = robo_shape_->get_complete_width_y();
        width = std::max( 4.f, ((width + inc)*100.f)/cell_width_ );

        float height = 0.f;
        height = robo_shape_->get_complete_width_x();
        height = std::max( 4.f, ((height + inc)*100.f)/cell_height_ );

        integrate_obstacle( posX, posY, width, height );

        old_readings_.push_back( new_readings_[i] );
      }
    }
  }
  delete pointsTransformed;
}

void
LaserOccupancyGrid::integrate_obstacle( int x, int y, int width, int height )
{
  std::vector< int > fast_obstacle = obstacle_map->get_obstacle( width, height, cfg_obstacle_inc_ );

  int posX = 0;
  int posY = 0;

  // i = x offset, i+1 = y offset, i+2 is cost
  for( unsigned int i = 0; i < fast_obstacle.size(); i+=3 ) {
    /* On the laser-points, we draw obstacles based on base_link. The obstacle has the robot-shape,
     * which means that we need to rotate the shape 180° around base_link and move that rotation-
     * point onto the laser-point on the grid. That's the same as adding the center_to_base_offset
     * to the calculated position of the obstacle-center ("x + fast_obstacle[i]" and "y" respectively).
     */
    posX = x + fast_obstacle[i]   + offset_base_.x;
    posY = y + fast_obstacle[i+1] + offset_base_.y;

    if( (posX > 0) && (posX < height_)
     && (posY > 0) && (posY < width_)
     && (occupancy_probs_[posX][posY] < fast_obstacle[i+2]) )
      {
      occupancy_probs_[posX][posY] = fast_obstacle[i+2];
    }
  }
}

} // namespace fawkes
