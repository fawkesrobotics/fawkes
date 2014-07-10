
/***************************************************************************
 *  og_laser.cpp - Occupancy grid for colli's A* search
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
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

#include <pcl/common/distances.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CLaserOccupancyGrid <plugins/colli/search/og_laser.h>
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
CLaserOccupancyGrid::CLaserOccupancyGrid( Laser360Interface * laser, Logger* logger, Configuration* config, tf::Transformer* listener,
                                          int width, int height,
                                          int cell_width, int cell_height)
 : OccupancyGrid( width, height, cell_width, cell_height )
{
  logger->log_debug("CLaserOccupancyGrid", "(Constructor): Entering");

  //read config
  std::string cfg_prefix = "/plugins/colli/";
  m_ObstacleDistance     = config->get_float((cfg_prefix + "laser_occupancy_grid/distance_account").c_str());
  m_InitialHistorySize  = 3*config->get_int((cfg_prefix + "laser_occupancy_grid/history/initial_size").c_str());
  m_MaxHistoryLength    = config->get_float((cfg_prefix + "laser_occupancy_grid/history/max_length").c_str());
  m_MinHistoryLength    = config->get_float((cfg_prefix + "laser_occupancy_grid/history/min_length").c_str());
  m_MinimumLaserLength  = config->get_float((cfg_prefix + "laser/min_reading_length").c_str());

  cfg_delete_invisible_old_obstacles_           = config->get_bool((cfg_prefix + "laser_occupancy_grid/history/delete_invisible_old_obstacles/enable").c_str());
  cfg_delete_invisible_old_obstacles_angle_min_ = config->get_int((cfg_prefix + "laser_occupancy_grid/history/delete_invisible_old_obstacles/angle_min").c_str());
  cfg_delete_invisible_old_obstacles_angle_max_ = config->get_int((cfg_prefix + "laser_occupancy_grid/history/delete_invisible_old_obstacles/angle_max").c_str());
  if ( cfg_delete_invisible_old_obstacles_angle_min_ >= 360 ) {
    logger_->log_warn("CLaserOccupancyGrid", "Min angle out of bounce, use 0");
    cfg_delete_invisible_old_obstacles_angle_min_ = 0;
  }
  if ( cfg_delete_invisible_old_obstacles_angle_min_ >= 360 ) {
    logger_->log_warn("CLaserOccupancyGrid", "Max angle out of bounce, use 360");
    cfg_delete_invisible_old_obstacles_angle_min_ = 360;
  }

  if (cfg_delete_invisible_old_obstacles_angle_max_ > cfg_delete_invisible_old_obstacles_angle_min_) {
    m_angle_range_ = deg2rad((unsigned int)abs(cfg_delete_invisible_old_obstacles_angle_max_ - cfg_delete_invisible_old_obstacles_angle_min_));
  } else {
    m_angle_range_ = deg2rad((360 - cfg_delete_invisible_old_obstacles_angle_min_) + cfg_delete_invisible_old_obstacles_angle_max_);
  }
  m_angle_min_ = deg2rad( cfg_delete_invisible_old_obstacles_angle_min_ );


  m_reference_frame     = config->get_string((cfg_prefix + "frame/odometry").c_str());
  m_laser_frame         = config->get_string((cfg_prefix + "frame/laser").c_str());       //TODO change to base_link => search in base_link instead base_laser

  cfg_obstacle_inc_           = config->get_bool((cfg_prefix + "obstacle_increasement").c_str());
  cfg_force_elipse_obstacle_  = config->get_bool((cfg_prefix + "laser_occupancy_grid/force_ellipse_obstacle").c_str());

  m_if_buffer_size      = config->get_int((cfg_prefix + "laser_occupancy_grid/buffer_size").c_str());
  m_if_buffer_size = std::max(m_if_buffer_size, 1); //needs to be >= 1, because the data is always wrote into the buffer (instead of read())

  cell_costs_.occ       = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/occupied").c_str());
  cell_costs_.near      = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/near").c_str());
  cell_costs_.mid       = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/mid").c_str());
  cell_costs_.far       = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/far").c_str());
  cell_costs_.free      = config->get_int((cfg_prefix + "laser_occupancy_grid/cell_cost/free").c_str());

  m_if_buffer_filled.resize(m_if_buffer_size);
  std::fill(m_if_buffer_filled.begin(), m_if_buffer_filled.end(), false);

  if_laser_   = laser;
  if_laser_->resize_buffers( m_if_buffer_size );
  logger_     = logger;
  tf_listener = listener;

  m_pRoboShape = new CRoboShape_Colli( (cfg_prefix + "roboshape/").c_str(), logger, config );
  m_vOldReadings.clear();
  initGrid();

  logger->log_debug("CLaserOccupancyGrid", "Generating obstacle map");
  bool obstacle_shape = m_pRoboShape->IsAngularRobot() && ! cfg_force_elipse_obstacle_;
  obstacle_map = new ColliObstacleMap(cell_costs_, obstacle_shape);
  logger->log_debug("CLaserOccupancyGrid", "Generating obstacle map done");

  m_LaserPosition = point_t(0,0);

  // calculate laser offset from robot center
  offset_base_.x=0;
  offset_base_.y=0;
  offset_laser_.x = m_pRoboShape->GetCompleteWidthX()/2.f - m_pRoboShape->GetRobotLengthforDegree(0);
  offset_laser_.y = m_pRoboShape->GetCompleteWidthY()/2.f - m_pRoboShape->GetRobotLengthforDegree(90);
  logger->log_debug("CLaserOccupancyGrid", "Laser (x,y) offset from robo-center is (%f, %f)",
                    offset_laser_.x, offset_laser_.y);


  logger->log_debug("CLaserOccupancyGrid", "(Constructor): Exiting");
}

/** Descturctor. */
CLaserOccupancyGrid::~CLaserOccupancyGrid()
{
  delete m_pRoboShape;
}

/** Reset all old readings and forget about the world state! */
void
CLaserOccupancyGrid::ResetOld()
{
  m_vOldReadings.clear();
  m_vOldReadings.reserve( m_InitialHistorySize );
}

/**
 * Gets data from laser (does! read it) and transforms them into the reference-frame (odom)
 */
void
CLaserOccupancyGrid::updateLaser()
{
  //check for free pos in buffer
  int if_buffer_free_pos = -1;

  for (int i = 0; i < m_if_buffer_size; ++i) {    //for all buffer possition
    if (m_if_buffer_filled[i] == false) {         //if free (used == false)
      if_buffer_free_pos = i;                     //use this buffer
      //stop loop
    }
  }
  //write BB date into buffer (instead of read())
  if ( if_buffer_free_pos < 0 ) {                 //if there is no free buffer
    logger_->log_error("CLaserOccupancyGrid", "if_laser buffer is full empty oldest");

                                                  //search for the oldest buffer and uses this
    double if_buffer_oldest_time = fawkes::Clock::instance()->now().in_sec() + 1000;
    int if_buffer_oldest_pos = -1;

    for (int i = 0; i < m_if_buffer_size; ++i) {
      if (if_laser_->buffer_timestamp( i ).in_sec() < if_buffer_oldest_time) {
        if_buffer_oldest_pos = i;
        if_buffer_oldest_time = if_laser_->buffer_timestamp( i ).in_sec();
      }
    }
    if_buffer_free_pos = if_buffer_oldest_pos;
  }

  if_laser_->copy_shared_to_buffer( if_buffer_free_pos );     //read new laser data
  m_if_buffer_filled[ if_buffer_free_pos ] = true;            //set buffer used

  m_vNewReadings.clear();
  m_vNewReadings.reserve( if_laser_->maxlenof_distances() * m_if_buffer_size );
  //for all buffer: try to transform and save in grid
  for (int i = 0; i < m_if_buffer_size; ++i) {
    if (m_if_buffer_filled[i] == true) {      //if is filled

      if_laser_->read_from_buffer( i );         //read buffer
      m_if_buffer_filled[i] = false;            //show buffer is not used
      //TODO just if there are new data
      const Time* laser_time  = if_laser_->timestamp();
      std::string laser_frame = if_laser_->frame();

      tf::StampedTransform transform;

      try {
        tf_listener->lookup_transform(m_reference_frame, laser_frame, laser_time, transform);

        tf::Vector3 pos_robot_tf = transform.getOrigin();
        cart_coord_2d_t pos_robot(pos_robot_tf.getX(), pos_robot_tf.getY());

        double angle_inc = M_PI * 2. / 360.;
        tf::Point p;
        //Save all Points in refernce Frame
        for (unsigned int i = 0; i < if_laser_->maxlenof_distances(); ++i) {
          if (if_laser_->distances(i) >= m_MinimumLaserLength) {
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

            CLaserOccupancyGrid::LaserPoint point;
            point.coord     = cart_coord_2d_t( p.getX(), p.getY() );
            point.timestamp = Time(laser_time);

            m_vNewReadings.push_back(point);

            if ( cfg_delete_invisible_old_obstacles_ ) {
              float angle_dist = angle_distance( m_angle_min_, point_polar.phi );
              if ( angle_dist >= 0 && angle_dist <= m_angle_range_ ) {
                validate_old_laser_points(pos_robot, point.coord);
              }
            }
          }
        }
      } catch(Exception &e) {
        m_if_buffer_filled[i] = true;            //show buffer still needs to be there
        logger_->log_debug("CLaserOccupancyGrid", "Unable to transform %s to %s. Laser-data not used, will keeped in history.",
                laser_frame.c_str(), m_reference_frame.c_str());
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
CLaserOccupancyGrid::validate_old_laser_points(cart_coord_2d_t pos_robot, cart_coord_2d_t pos_new_laser_point)
{
  std::vector< LaserPoint > old_readings_tmp;

  Eigen::Vector4f pol(pos_robot.x, pos_robot.y, 0, 0);
  Eigen::Vector4f ld(pos_new_laser_point.x - pos_robot.x , pos_new_laser_point.y - pos_robot.y, 0, 0);
  double d_new = sqrt(ld[0]*ld[0] + ld[1]*ld[1]);

  for ( std::vector< LaserPoint >::iterator it = m_vOldReadings.begin();
      it != m_vOldReadings.end(); ++it ) {

    //calculate distance between "old point" and "ray trace line of new point"
    Eigen::Vector4f point((*it).coord.x, (*it).coord.y, 0, 0);

    double d = sqrt( pcl::sqrPointToLineDistance( point, pol, ld ) );

    cart_coord_2d_t point_old = (*it).coord;

    if ( d < 0.003 ) {                                                          // if distance to line is in a threashold. ( arcsin(1°) * 0.2 > 0.003 (interferiance with next laser))
      float x_p, y_p;                                                           //   calculate if old point is before or behind new point
      x_p = point_old.x - pos_robot.x;                                          //   (from the view of the actual robot possition)
      y_p = point_old.y - pos_robot.y;
      float d_old = sqrt(x_p*x_p + y_p*y_p);

      if ( ( d_new <= d_old + m_ObstacleDistance )                              // if d new < d old => old in shaddow, so keep it
          || (x_p >= 0) != (ld[0] >= 0)                                         // or x or y are not in the same directions of the robot
          || (y_p >= 0) != (ld[1] >= 0) ) {                                     // ( opposite beams => no realation)
        old_readings_tmp.push_back( *it );
      }                                                                         // non existing else: old not in shaddow => you can see throu it, so don't keep it
    } else {                                                                    // if the old point is not close to the line (no relation)
      old_readings_tmp.push_back( *it );
    }
  }

  m_vOldReadings.clear();
  m_vOldReadings.reserve( old_readings_tmp.size() );

  for (unsigned int i = 0; i < old_readings_tmp.size(); ++i) {
    m_vOldReadings.push_back( old_readings_tmp[i] );
  }
}

float
CLaserOccupancyGrid::obstacle_in_path_distance( float vx, float vy )
{
  if_laser_->read();
  int angle = roundf( rad2deg( normalize_rad( atan2f(vy, vx) ) ) );

  float distance_min = 10;

  int cfg_beams = 11;

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
CLaserOccupancyGrid::UpdateOccGrid( int midX, int midY, float inc, float vx, float vy )
{
  float vel = std::sqrt(vx*vx + vy*vy);

  float next_obstacle = obstacle_in_path_distance( vx, vy );

  m_LaserPosition.x = midX;
  m_LaserPosition.y = midY;

  for ( int y = 0; y < m_Width; ++y )
    for ( int x = 0; x < m_Height; ++x )
      m_OccupancyProb[x][y] = cell_costs_.free;

  updateLaser();

  tf::StampedTransform transform;

  try {
    tf_listener->lookup_transform(m_laser_frame, m_reference_frame, Time(0,0), transform);

  } catch(Exception &e) {
    logger_->log_error("CLaserOccupancyGrid", "Unable to transform %s to %s. Can't put obstacles into the grid",
        m_reference_frame.c_str(), m_laser_frame.c_str());
    return 0.;
  }

  IntegrateOldReadings( midX, midY, inc, vel, transform );
  IntegrateNewReadings( midX, midY, inc, vel, transform );

  return next_obstacle;
}

/**
 * Transforms all given points with the given transform
 * @param laserPoints vector of LaserPoint, that contains the points to transform
 * @param transform stamped transform, the transform to transform with
 * @return the transformed laserPoints
 */
std::vector< CLaserOccupancyGrid::LaserPoint >*
CLaserOccupancyGrid::transformLaserPoints(std::vector< CLaserOccupancyGrid::LaserPoint >& laserPoints, tf::StampedTransform& transform)
{
  int count_points = laserPoints.size();
  std::vector< CLaserOccupancyGrid::LaserPoint >* laserPointsTransformed = new std::vector< CLaserOccupancyGrid::LaserPoint >();
  laserPointsTransformed->reserve( count_points );

  tf::Point p;

  for (int i = 0; i < count_points; ++i) {
    p.setValue(laserPoints[i].coord.x, laserPoints[i].coord.y, 0.);
    p = transform * p;

    CLaserOccupancyGrid::LaserPoint point;
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
CLaserOccupancyGrid::GetLaserPosition()
{
  return m_LaserPosition;
}

/** Set the offset of base_link from laser.
 * @param x offset in x-direction (in meters)
 * @param y offset in y-direction (in meters)
 */
void
CLaserOccupancyGrid::set_base_offset(float x, float y)
{
  offset_base_.x = (int)round( (offset_laser_.x + x)*100.f/m_CellHeight ); // # in grid-cells
  offset_base_.y = (int)round( (offset_laser_.y + y)*100.f/m_CellWidth  );
}


/** Get cell costs.
 * @return struct that contains all the cost values for the occgrid cells
 */
colli_cell_cost_t
CLaserOccupancyGrid::get_cell_costs() const
{
  return cell_costs_;
}

void
CLaserOccupancyGrid::IntegrateOldReadings( int midX, int midY, float inc, float vel,
                                           tf::StampedTransform& transform )
{
  std::vector< CLaserOccupancyGrid::LaserPoint > old_readings;
  old_readings.reserve( m_vOldReadings.size() );
  std::vector< CLaserOccupancyGrid::LaserPoint >* pointsTransformed = transformLaserPoints(m_vOldReadings, transform);

  float newpos_x, newpos_y;

  Clock* clock = Clock::instance();
  Time history = Time(clock) - Time(double(std::max( m_MinHistoryLength, m_MaxHistoryLength)));

  // update all old readings
  for ( unsigned int i = 0; i < pointsTransformed->size(); ++i ) {

    if ( (*pointsTransformed)[i].timestamp.in_sec() >= history.in_sec() ) {

      newpos_x =  (*pointsTransformed)[i].coord.x;
      newpos_y =  (*pointsTransformed)[i].coord.y;

      //newpos_x =  m_vOldReadings[i].coord.x + xref;
      //newpos_y =  m_vOldReadings[i].coord.y + yref;

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
        int posX = midX + (int)((newpos_x*100.f) / ((float)m_CellHeight ));
        int posY = midY + (int)((newpos_y*100.f) / ((float)m_CellWidth ));
        if( posX > 4 && posX < m_Height-5
         && posY > 4 && posY < m_Width-5 )
          {
          old_readings.push_back( m_vOldReadings[i] );

          // 25 cm's in my opinion, that are here: 0.25*100/m_CellWidth
          //int size = (int)(((0.25f+inc)*100.f)/(float)m_CellWidth);
          float width = m_pRoboShape->GetCompleteWidthY();
          width = std::max( 4.f, ((width + inc)*100.f)/m_CellWidth );
          float height = m_pRoboShape->GetCompleteWidthX();
          height = std::max( 4.f, ((height + inc)*100.f)/m_CellHeight );
          integrateObstacle( posX, posY, width, height );
        }
      }

    }
  }

  m_vOldReadings.clear();
  m_vOldReadings.reserve( old_readings.size() );

  // integrate the new calculated old readings
  for ( unsigned int i = 0; i < old_readings.size(); i++ )
    m_vOldReadings.push_back( old_readings[i] );

  delete pointsTransformed;
}


void
CLaserOccupancyGrid::IntegrateNewReadings( int midX, int midY, float inc, float vel,
                                           tf::StampedTransform& transform )
{
  std::vector< CLaserOccupancyGrid::LaserPoint >* pointsTransformed = transformLaserPoints(m_vNewReadings, transform);

  int numberOfReadings = pointsTransformed->size();
  //TODO resize, reserve??

  int posX, posY;
  cart_coord_2d_t point;
  float oldp_x = 1000.f;
  float oldp_y = 1000.f;

  for ( int i = 0; i < numberOfReadings; i++ ) {
    point = (*pointsTransformed)[i].coord;

    if( sqrt(sqr(point.x) + sqr(point.y)) >= m_MinimumLaserLength
     && distance(point.x, point.y, oldp_x, oldp_y) >= m_ObstacleDistance)
      {
      oldp_x = point.x;
      oldp_y = point.y;
      posX = midX + (int)((point.x*100.f) / ((float)m_CellHeight ));
      posY = midY + (int)((point.y*100.f) / ((float)m_CellWidth ));

      if ( !( posX <= 5 || posX >= m_Height-6 || posY <= 5 || posY >= m_Width-6 ) ) {
        float width = 0.f;
        width = m_pRoboShape->GetCompleteWidthY();
        width = std::max( 4.f, ((width + inc)*100.f)/m_CellWidth );

        float height = 0.f;
        height = m_pRoboShape->GetCompleteWidthX();
        height = std::max( 4.f, ((height + inc)*100.f)/m_CellHeight );

        integrateObstacle( posX, posY, width, height );

        m_vOldReadings.push_back( m_vNewReadings[i] );
      }
    }
  }
  delete pointsTransformed;
}

void
CLaserOccupancyGrid::integrateObstacle( int x, int y, int width, int height )
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

    if( (posX > 0) && (posX < m_Height)
     && (posY > 0) && (posY < m_Width)
     && (m_OccupancyProb[posX][posY] < fast_obstacle[i+2]) )
      {
      m_OccupancyProb[posX][posY] = fast_obstacle[i+2];
    }
  }
}

} // namespace fawkes
