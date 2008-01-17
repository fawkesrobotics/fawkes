
/***************************************************************************
 *  kicker.cpp - Fawkes BlackBoard Interface - KickerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#include <interfaces/kicker.h>

#include <string.h>
#include <stdlib.h>

/** @class KickerInterface interfaces/kicker.h
 * KickerInterface Fawkes BlackBoard Interface.
 * 
      In these variables it is stored how often the right, center or 
      left kick have been triggered.
    
 */



/** Constructor */
KickerInterface::KickerInterface() : Interface()
{
  data_size = sizeof(KickerInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (KickerInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
}
/** Destructor */
KickerInterface::~KickerInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get num_kicks_left value.
 * 
      Number of Left-Kicks
    
 * @return num_kicks_left value
 */
int
KickerInterface::num_kicks_left()
{
  return data->num_kicks_left;
}

/** Set num_kicks_left value.
 * 
      Number of Left-Kicks
    
 * @param new_num_kicks_left new num_kicks_left value
 */
void
KickerInterface::set_num_kicks_left(const int new_num_kicks_left)
{
  data->num_kicks_left = new_num_kicks_left;
}

/** Get num_kicks_center value.
 * 
      Number of Center-Kicks
    
 * @return num_kicks_center value
 */
int
KickerInterface::num_kicks_center()
{
  return data->num_kicks_center;
}

/** Set num_kicks_center value.
 * 
      Number of Center-Kicks
    
 * @param new_num_kicks_center new num_kicks_center value
 */
void
KickerInterface::set_num_kicks_center(const int new_num_kicks_center)
{
  data->num_kicks_center = new_num_kicks_center;
}

/** Get num_kicks_right value.
 * 
      Number of Right-Kicks
    
 * @return num_kicks_right value
 */
int
KickerInterface::num_kicks_right()
{
  return data->num_kicks_right;
}

/** Set num_kicks_right value.
 * 
      Number of Right-Kicks
    
 * @param new_num_kicks_right new num_kicks_right value
 */
void
KickerInterface::set_num_kicks_right(const int new_num_kicks_right)
{
  data->num_kicks_right = new_num_kicks_right;
}

/** Get guide_ball_side value.
 * Side where the ball
      guidance arm is currently erected.
 * @return guide_ball_side value
 */
KickerInterface::GuideBallSideEnum
KickerInterface::guide_ball_side()
{
  return data->guide_ball_side;
}

/** Set guide_ball_side value.
 * Side where the ball
      guidance arm is currently erected.
 * @param new_guide_ball_side new guide_ball_side value
 */
void
KickerInterface::set_guide_ball_side(const GuideBallSideEnum new_guide_ball_side)
{
  data->guide_ball_side = new_guide_ball_side;
}

/** Get current_intensity value.
 * 
      The currently set intensity.
    
 * @return current_intensity value
 */
unsigned int
KickerInterface::current_intensity()
{
  return data->current_intensity;
}

/** Set current_intensity value.
 * 
      The currently set intensity.
    
 * @param new_current_intensity new current_intensity value
 */
void
KickerInterface::set_current_intensity(const unsigned int new_current_intensity)
{
  data->current_intensity = new_current_intensity;
}

/* =========== messages =========== */
/** @class KickerInterface::KickMessage interfaces/kicker.h
 * KickMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_left initial value for left
 * @param ini_center initial value for center
 * @param ini_right initial value for right
 * @param ini_intensity initial value for intensity
 */
KickerInterface::KickMessage::KickMessage(bool ini_left, bool ini_center, bool ini_right, unsigned int ini_intensity) : Message()
{
  data_size = sizeof(KickMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (KickMessage_data_t *)data_ptr;
  data->left = ini_left;
  data->center = ini_center;
  data->right = ini_right;
  data->intensity = ini_intensity;
}
/** Constructor */
KickerInterface::KickMessage::KickMessage() : Message()
{
  data_size = sizeof(KickMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (KickMessage_data_t *)data_ptr;
}
/** Destructor */
KickerInterface::KickMessage::~KickMessage()
{
}
/* Methods */
/** Get left value.
 * True to kick with left kicker.
 * @return left value
 */
bool
KickerInterface::KickMessage::is_left()
{
  return data->left;
}

/** Set left value.
 * True to kick with left kicker.
 * @param new_left new left value
 */
void
KickerInterface::KickMessage::set_left(const bool new_left)
{
  data->left = new_left;
}

/** Get center value.
 * True to kick with central kicker.
 * @return center value
 */
bool
KickerInterface::KickMessage::is_center()
{
  return data->center;
}

/** Set center value.
 * True to kick with central kicker.
 * @param new_center new center value
 */
void
KickerInterface::KickMessage::set_center(const bool new_center)
{
  data->center = new_center;
}

/** Get right value.
 * True to kick with right kicker.
 * @return right value
 */
bool
KickerInterface::KickMessage::is_right()
{
  return data->right;
}

/** Set right value.
 * True to kick with right kicker.
 * @param new_right new right value
 */
void
KickerInterface::KickMessage::set_right(const bool new_right)
{
  data->right = new_right;
}

/** Get intensity value.
 * Intensity in the range [0..255].
 * @return intensity value
 */
unsigned int
KickerInterface::KickMessage::intensity()
{
  return data->intensity;
}

/** Set intensity value.
 * Intensity in the range [0..255].
 * @param new_intensity new intensity value
 */
void
KickerInterface::KickMessage::set_intensity(const unsigned int new_intensity)
{
  data->intensity = new_intensity;
}

/** @class KickerInterface::ResetCounterMessage interfaces/kicker.h
 * ResetCounterMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
KickerInterface::ResetCounterMessage::ResetCounterMessage() : Message()
{
  data_size = sizeof(ResetCounterMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ResetCounterMessage_data_t *)data_ptr;
}
/** Destructor */
KickerInterface::ResetCounterMessage::~ResetCounterMessage()
{
}
/* Methods */
/** @class KickerInterface::GuideBallMessage interfaces/kicker.h
 * GuideBallMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_guide_ball_side initial value for guide_ball_side
 */
KickerInterface::GuideBallMessage::GuideBallMessage(GuideBallSideEnum ini_guide_ball_side) : Message()
{
  data_size = sizeof(GuideBallMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GuideBallMessage_data_t *)data_ptr;
  data->guide_ball_side = ini_guide_ball_side;
}
/** Constructor */
KickerInterface::GuideBallMessage::GuideBallMessage() : Message()
{
  data_size = sizeof(GuideBallMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GuideBallMessage_data_t *)data_ptr;
}
/** Destructor */
KickerInterface::GuideBallMessage::~GuideBallMessage()
{
}
/* Methods */
/** Get guide_ball_side value.
 * Side where to guide the ball and erect the arm.
 * @return guide_ball_side value
 */
KickerInterface::GuideBallSideEnum
KickerInterface::GuideBallMessage::guide_ball_side()
{
  return data->guide_ball_side;
}

/** Set guide_ball_side value.
 * Side where to guide the ball and erect the arm.
 * @param new_guide_ball_side new guide_ball_side value
 */
void
KickerInterface::GuideBallMessage::set_guide_ball_side(const GuideBallSideEnum new_guide_ball_side)
{
  data->guide_ball_side = new_guide_ball_side;
}

/** Check if message is valid an can be queued.
 * @param message Message to check
 */
bool
KickerInterface::messageValid(const Message *message) const
{
  const KickMessage *m0 = dynamic_cast<const KickMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const ResetCounterMessage *m1 = dynamic_cast<const ResetCounterMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const GuideBallMessage *m2 = dynamic_cast<const GuideBallMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(KickerInterface)
/// @endcond

