
/***************************************************************************
 *  conversion.h - conversion between encoder and radian motor values
 *
 *  Created: Thu Dec 02 13:51:46 2010
 *  Copyright  2010  Bahram Maleki-Fard
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __PLUGINS_KATANA_CONVERSION_H
#define __PLUGINS_KATANA_CONVERSION_H

#include <utils/math/angle.h>
#include <common/MathHelperFunctions.h>

#include <vector>
using namespace KNI_MHF;

namespace fawkes {

/** Convert radian vaulues of katana arm to encoder values
 * @param rad vector with radian angle values
 * @param enc vector to be filled with encoder values
 */
inline void
radToEnc(std::vector<float>& rad, std::vector<int>& enc)
{
  enc.clear();

  enc.push_back(rad2enc(rad[0], deg2rad(  6.65f),  51200,  31000,  1));
  enc.push_back(rad2enc(rad[1], deg2rad(124.25f),  94976, -31000,  1));
  enc.push_back(rad2enc(rad[2], deg2rad( 52.70f),  81408, -31000, -1));
  enc.push_back(rad2enc(rad[3], deg2rad( 63.50f),  51200,  31000,  1));
  enc.push_back(rad2enc(rad[4], deg2rad(  8.50f),  51200,  31000,  1));
}

/** Convert encoder vaulues of katana arm to radian angles.
 * @param enc vector with encoder values, received from CKatana::getRobotEncoders
 * @param rad vector to be filled with angle values
 */
inline void
encToRad(std::vector<int>& enc, std::vector<float>& rad)
{
  rad.clear();

  rad.push_back(enc2rad(enc[0], deg2rad(  6.65f),  51200,  31000,  1));
  rad.push_back(enc2rad(enc[1], deg2rad(124.25f),  94976, -31000,  1));
  rad.push_back(enc2rad(enc[2], deg2rad( 52.70f),  81408, -31000, -1));
  rad.push_back(enc2rad(enc[3], deg2rad( 63.50f),  51200,  31000,  1));
  rad.push_back(enc2rad(enc[4], deg2rad(  8.50f),  51200,  31000,  1));
}

} // end namespace fawkes

#endif
