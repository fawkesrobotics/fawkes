
/***************************************************************************
 *  robo_laserpoint.h - Class for one laser point
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_UTILS_ROB_ROBO_LASERPOINT_H_
#define __PLUGINS_COLLI_UTILS_ROB_ROBO_LASERPOINT_H_

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Reading struct.*/
struct Reading
{
  float posX;     /**< x position of reading */
  float posY;     /**< y position of reading */
  float length;   /**< length of reading */
  float rad;      /**< angle of reading (in rad) */
};

class LaserPoint
{
 public:

  LaserPoint( int numberOfReadings ) throw (int);
  ~LaserPoint();

  ///\brief Returns the number-reading length.
  float  GetLength( int number );

  ///\brief Returns the number-reading radians.
  float  GetRadians  ( int number );

  ///\brief Returns the number-reading x coordinate.
  float  GetPosX  ( int number );

  ///\brief Returns the number-reading y coordinate.
  float  GetPosY  ( int number );

  ///\brief Sets the number-readings length.
  void  SetLength  ( int number, float length );

  ///\brief Sets the number-readings angle.
  void SetRadians( int number, float radians );

  ///\brief Sets the number-readings x coordinate.
  void  SetPosX  ( int number, float posX);

  ///\brief Computes the number-readings x coordinate by the number-readings
  void  SetPosX  ( int number);

  ///\brief Sets the number-readings x coordinate.
  void  SetPosY  ( int number, float posY );

  ///\brief Computes the number-readings x coordinate by the number-readings
  void  SetPosY  ( int number );

  ///\brief Computes the number-readings coordinates by the number-readings
  void  SetPos  ( int number );

 private:

  // array containing scan data
  std::vector<Reading> m_pLaserPoint;

  // number of readings
  int m_NumberOfReadings;

  // range check methods
  int  RangeCheck  ( int number);

};

} // namespace fawkes

#endif
