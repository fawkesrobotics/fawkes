
/***************************************************************************
 *  color.h - Abstract class defining a camera color controller
 *
 *  Created: Wed Apr 22 11:19:04 2009
 *  Copyright  2009      Tobias Kellner
 *             2005-2009 Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CAMS_CONTROL_COLOR_H_
#define __FIREVISION_CAMS_CONTROL_COLOR_H_

#include <fvcams/control/control.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraControlColor : virtual public CameraControl
{
 public:
  virtual ~CameraControlColor();

  virtual bool         auto_gain()                                      = 0;
  virtual void         set_auto_gain(bool enabled)                      = 0;
  virtual bool         auto_white_balance()                             = 0;
  virtual void         set_auto_white_balance(bool enabled)             = 0;
  virtual unsigned int exposure_auto()                                  = 0;
  virtual void         set_exposure_auto(unsigned int enabled)          = 0;
  virtual void         set_auto_all(bool enabled);

  virtual int          red_balance()                                    = 0;
  virtual void         set_red_balance(int red_balance)                 = 0;
  virtual int          blue_balance()                                   = 0;
  virtual void         set_blue_balance(int blue_balance)               = 0;

  virtual int          u_balance()                                      = 0;
  virtual void         set_u_balance(int u_balance)                     = 0;
  virtual int          v_balance()                                      = 0;
  virtual void         set_v_balance(int v_balance)                     = 0;

  virtual unsigned int brightness()                                     = 0;
  virtual void         set_brightness(unsigned int brightness)          = 0;
  virtual unsigned int contrast()                                       = 0;
  virtual void         set_contrast(unsigned int contrast)              = 0;
  virtual unsigned int saturation()                                     = 0;
  virtual void         set_saturation(unsigned int saturation)          = 0;
  virtual int          hue()                                            = 0;
  virtual void         set_hue(int hue)                                 = 0;
  virtual unsigned int exposure()                                       = 0;
  virtual void         set_exposure(unsigned int exposure)              = 0;
  virtual unsigned int gain()                                           = 0;
  virtual void         set_gain(unsigned int gain)                      = 0;
};

} // end namespace firevision

#endif // __FIREVISION_CAMS_CONTROL_COLOR_H_
