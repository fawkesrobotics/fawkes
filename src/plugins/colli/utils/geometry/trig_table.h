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
 * <Stefan_J@gmx.de>
 *
 * Containing Implementation class trigonometry table.
 *
 */

/***********************************************************************
 *
 * $Id$
 *
 * Description: Contains the implementation for putting laser scans in a
 *    Occupancy Grid (which is the father of this object).
 *
 * last modified: $Date$
 *            by: $Author$
 *
 **********************************************************************/

#ifndef _COLLI_TRIG_TABLE_H_
#define _COLLI_TRIG_TABLE_H_

#include <utils/math/angle.h>

#include <cmath>
#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class TrigTable
{

 public:

  /** @param Resolution is the number of steps per degree!
   *  @doc e.G. TrigTable( 4 ) means 4 steps per each degree.
   */
  TrigTable( int resolution_per_degree );

  /** destruct object
   */
  ~TrigTable();

  /** return linear interpolated sinus.
   */
  float GetSin( float radians );

  /** return linear interpolated cosinus.
   */
  float GetCos( float radians );


 private:

  std::vector< float > m_vSinTable;
  std::vector< float > m_vCosTable;

  float InterpolateSin( float index_to_interpolate );
  float InterpolateCos( float index_to_interpolate );
  int m_Resolution;
};


inline
TrigTable::TrigTable( int resolution_per_degree )
{
  m_Resolution = resolution_per_degree;
  m_vSinTable.reserve( 360*resolution_per_degree );
  m_vCosTable.reserve( 360*resolution_per_degree );

  for ( int i = 0; i < 360*resolution_per_degree; i++ ) {
    m_vSinTable.push_back( sin( (i*M_PI)/(180.0*resolution_per_degree) ) );
    m_vCosTable.push_back( cos( (i*M_PI)/(180.0*resolution_per_degree) ) );
  }
}


inline
TrigTable::~TrigTable( )
{
  m_vSinTable.clear();
  m_vCosTable.clear();
}


inline float
TrigTable::GetSin( float radians )
{
  radians = normalize_rad( radians );
  float degree = rad2deg( radians ) * m_Resolution;
  int index = (int)degree;
  if ( index - degree == 0 )
    return m_vSinTable[index];
  else
    return InterpolateSin( degree );
}


inline float
TrigTable::GetCos( float radians )
{
  radians = normalize_rad( radians );
  float degree = rad2deg( radians ) * m_Resolution;
  int index = (int)degree;
  if ( index - degree == 0 )
    return m_vCosTable[index];
  else
    return InterpolateCos( degree );

}


inline float
TrigTable::InterpolateSin( float index_to_interpolate )
{
  float left = index_to_interpolate - (int)index_to_interpolate;
  float right = 1-left;
  return ( right* m_vSinTable[(int)index_to_interpolate] +
           left * m_vSinTable[((int)index_to_interpolate+1)%(360*m_Resolution)] );
}


inline float
TrigTable::InterpolateCos( float index_to_interpolate )
{
  float left = index_to_interpolate - (int)index_to_interpolate;
  float right = 1-left;
  return ( right* m_vCosTable[(int)index_to_interpolate] +
           left * m_vCosTable[((int)index_to_interpolate+1)%(360*m_Resolution)] );
}

} // namespace fawkes

#endif
