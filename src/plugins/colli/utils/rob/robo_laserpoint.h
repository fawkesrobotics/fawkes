/*
 ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
 ©                                                                            ©
 ©                                            ####   ####           .-""-.    ©
 ©       # #                             #   #    # #    #         /[] _ _\   ©
 ©       # #                                 #    # #             _|_o_LII|_  ©
 © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
 © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
 © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
 © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
 © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
 ©                                                               /__|    |__\ ©
 ©                                                                            ©
 ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/

/* Written by Stefan Jacobs
 * for module Colli-A*
 *
 * Containing Header for one laser point interface.
 *
 */

/***********************************************************************
 *
 * $Id$
 *
 * Description: Contains a class for handling laser scans
 *
 *
 * last modified: $Date$
 *            by: $Author$
 *
 **********************************************************************/

#ifndef _COLLI_UTILS_ROB_ROBO_LASERPOINT_H_
#define _COLLI_UTILS_ROB_ROBO_LASERPOINT_H_

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
