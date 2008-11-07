
/***************************************************************************
 *  KickerInterface.cpp - Fawkes BlackBoard Interface - KickerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
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

#include "KickerInterface.h"

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class KickerInterface <interfaces/KickerInterface.h>
 * KickerInterface Fawkes BlackBoard Interface.
 * 
      In these variables it is stored how often the right, center or 
      left kick have been triggered.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
KickerInterface::KickerInterface() : Interface()
{
  data_size = sizeof(KickerInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (KickerInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(Interface::IFT_INT, "num_kicks_left", 1, &data->num_kicks_left);
  add_fieldinfo(Interface::IFT_INT, "num_kicks_center", 1, &data->num_kicks_center);
  add_fieldinfo(Interface::IFT_INT, "num_kicks_right", 1, &data->num_kicks_right);
  add_fieldinfo(Interface::IFT_UINT, "current_intensity", 1, &data->current_intensity);
  unsigned char tmp_hash[] = {0xdc, 0xe9, 0x59, 0xc4, 0xc2, 0xd9, 0x46, 0x62, 0xd7, 0x78, 0x52, 0xb0, 0x6f, 0xb, 0x2c, 0x76};
  set_hash(tmp_hash);
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
KickerInterface::num_kicks_left() const
{
  return data->num_kicks_left;
}

/** Get maximum length of num_kicks_left value.
 * @return length of num_kicks_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KickerInterface::maxlenof_num_kicks_left() const
{
  return 1;
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
KickerInterface::num_kicks_center() const
{
  return data->num_kicks_center;
}

/** Get maximum length of num_kicks_center value.
 * @return length of num_kicks_center value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KickerInterface::maxlenof_num_kicks_center() const
{
  return 1;
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
KickerInterface::num_kicks_right() const
{
  return data->num_kicks_right;
}

/** Get maximum length of num_kicks_right value.
 * @return length of num_kicks_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KickerInterface::maxlenof_num_kicks_right() const
{
  return 1;
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
KickerInterface::guide_ball_side() const
{
  return data->guide_ball_side;
}

/** Get maximum length of guide_ball_side value.
 * @return length of guide_ball_side value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KickerInterface::maxlenof_guide_ball_side() const
{
  return 1;
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
KickerInterface::current_intensity() const
{
  return data->current_intensity;
}

/** Get maximum length of current_intensity value.
 * @return length of current_intensity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KickerInterface::maxlenof_current_intensity() const
{
  return 1;
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

/* =========== message create =========== */
Message *
KickerInterface::create_message(const char *type) const
{
  if ( strncmp("KickMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new KickMessage();
  } else if ( strncmp("ResetCounterMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ResetCounterMessage();
  } else if ( strncmp("GuideBallMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GuideBallMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/* =========== messages =========== */
/** @class KickerInterface::KickMessage <interfaces/KickerInterface.h>
 * KickMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_left initial value for left
 * @param ini_center initial value for center
 * @param ini_right initial value for right
 * @param ini_intensity initial value for intensity
 */
KickerInterface::KickMessage::KickMessage(const bool ini_left, const bool ini_center, const bool ini_right, const unsigned int ini_intensity) : Message("KickMessage")
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
KickerInterface::KickMessage::KickMessage() : Message("KickMessage")
{
  data_size = sizeof(KickMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (KickMessage_data_t *)data_ptr;
}

/** Destructor */
KickerInterface::KickMessage::~KickMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KickerInterface::KickMessage::KickMessage(const KickMessage *m) : Message("KickMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (KickMessage_data_t *)data_ptr;
}

/* Methods */
/** Get left value.
 * True to kick with left kicker.
 * @return left value
 */
bool
KickerInterface::KickMessage::is_left() const
{
  return data->left;
}

/** Get maximum length of left value.
 * @return length of left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KickerInterface::KickMessage::maxlenof_left() const
{
  return 1;
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
KickerInterface::KickMessage::is_center() const
{
  return data->center;
}

/** Get maximum length of center value.
 * @return length of center value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KickerInterface::KickMessage::maxlenof_center() const
{
  return 1;
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
KickerInterface::KickMessage::is_right() const
{
  return data->right;
}

/** Get maximum length of right value.
 * @return length of right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KickerInterface::KickMessage::maxlenof_right() const
{
  return 1;
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
KickerInterface::KickMessage::intensity() const
{
  return data->intensity;
}

/** Get maximum length of intensity value.
 * @return length of intensity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KickerInterface::KickMessage::maxlenof_intensity() const
{
  return 1;
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

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
KickerInterface::KickMessage::clone() const
{
  return new KickerInterface::KickMessage(this);
}
/** @class KickerInterface::ResetCounterMessage <interfaces/KickerInterface.h>
 * ResetCounterMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
KickerInterface::ResetCounterMessage::ResetCounterMessage() : Message("ResetCounterMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
KickerInterface::ResetCounterMessage::~ResetCounterMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
KickerInterface::ResetCounterMessage::ResetCounterMessage(const ResetCounterMessage *m) : Message("ResetCounterMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
KickerInterface::ResetCounterMessage::clone() const
{
  return new KickerInterface::ResetCounterMessage(this);
}
/** @class KickerInterface::GuideBallMessage <interfaces/KickerInterface.h>
 * GuideBallMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_guide_ball_side initial value for guide_ball_side
 */
KickerInterface::GuideBallMessage::GuideBallMessage(const GuideBallSideEnum ini_guide_ball_side) : Message("GuideBallMessage")
{
  data_size = sizeof(GuideBallMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GuideBallMessage_data_t *)data_ptr;
  data->guide_ball_side = ini_guide_ball_side;
}
/** Constructor */
KickerInterface::GuideBallMessage::GuideBallMessage() : Message("GuideBallMessage")
{
  data_size = sizeof(GuideBallMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GuideBallMessage_data_t *)data_ptr;
}

/** Destructor */
KickerInterface::GuideBallMessage::~GuideBallMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KickerInterface::GuideBallMessage::GuideBallMessage(const GuideBallMessage *m) : Message("GuideBallMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GuideBallMessage_data_t *)data_ptr;
}

/* Methods */
/** Get guide_ball_side value.
 * Side where to guide the ball and erect the arm.
 * @return guide_ball_side value
 */
KickerInterface::GuideBallSideEnum
KickerInterface::GuideBallMessage::guide_ball_side() const
{
  return data->guide_ball_side;
}

/** Get maximum length of guide_ball_side value.
 * @return length of guide_ball_side value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KickerInterface::GuideBallMessage::maxlenof_guide_ball_side() const
{
  return 1;
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

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
KickerInterface::GuideBallMessage::clone() const
{
  return new KickerInterface::GuideBallMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
KickerInterface::message_valid(const Message *message) const
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


} // end namespace fawkes
