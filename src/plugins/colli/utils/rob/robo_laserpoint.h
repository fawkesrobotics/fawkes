
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

/* Reading struct.
 */
struct Reading
{
  float posX, posY, length, rad;
};

class TrigTable;

/** Ths class contains methods for handling scan data.
 */
class LaserPoint
{
 public:

  /** Constructor.
   * @param numberOfReadings is the number of scan-readings the scan should hold.
   * @param * dbg is the an instance of the Debug-Class.
   *  \exception (int 1) the could not be instanced.
   */
  LaserPoint( int numberOfReadings ) throw (int);


  /** Destructor.
   */
  ~LaserPoint();

  /** Returns the number-reading length.
   *  @param number is the readings number.
   *  @return float is the length.
   */
  float  GetLength( int number );

  /** Returns the number-reading x coordinate.
   *  @param number is the readings number.
   *  @return float is the x coordinate.
   */
  float  GetPosX  ( int number );

  /** Returns the number-reading y coordinate.
   *  @param number is the readings number.
   *  @return float is the y coordinate.
   */
  float  GetPosY  ( int number );

  /** Returns the number-reading radians.
   *  @param number is the readings number.
   *  @return float is the angle in rad.
   */
  float  GetRadians  ( int number );


  /** Sets the number-readings angle.
   *  @param number is the readings number.
   *  @param length is the readings angle in rad.
   */
  void SetRadians( int number, float radians );

  /** Sets the number-readings length.
   *  @param number is the readings number.
   *  @param length is the readings length.
   */
  void  SetLength  ( int number, float length );

  /** Sets the number-readings x coordinate.
   *  @param number is the readings number.
   *  @param posX is the readings x coordinate.
   */
  void  SetPosX  ( int number, float posX);

  /** Computes the number-readings x coordinate by
   *  the number-readings radians and length and
   *  finally sets the computed value.
   *  @param number is the readings number.
   */
  void  SetPosX  ( int number);

  /** Sets the number-readings x coordinate.
   *  @param number is the readings number.
   *  @param posY is the readings y coordinate.
   */
  void  SetPosY  ( int number, float posY );

  /** Computes the number-readings y coordinate by
   *  the number-readings radians and length and
   *  finally sets the computed value.
   *  @param number is the readings number.
   */
  void  SetPosY  ( int number );

  /** Computes the number-readings coordinates by
   *  the number-readings radians and length and
   *  finally sets the computed values.
   *  @param number is the readings number.
   */
  void  SetPos  ( int number );


  // ======================================================= //

 private:

  TrigTable * m_pTrigTable;

  // array containing scan data
  std::vector<Reading> m_pLaserPoint;

  // number of readings
  int m_NumberOfReadings;

  // range check methods
  int  RangeCheck  ( int number);

};

} // namespace fawkes

#endif
