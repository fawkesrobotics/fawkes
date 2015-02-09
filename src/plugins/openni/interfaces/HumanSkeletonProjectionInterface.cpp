
/***************************************************************************
 *  HumanSkeletonProjectionInterface.cpp - Fawkes BlackBoard Interface - HumanSkeletonProjectionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007-2011  Tim Niemueller
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

#include <interfaces/HumanSkeletonProjectionInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class HumanSkeletonProjectionInterface <interfaces/HumanSkeletonProjectionInterface.h>
 * HumanSkeletonProjectionInterface Fawkes BlackBoard Interface.
 * 
      This interface provides projections of the 3D position coordinates into the 2D image of
      the acquiring camera. Instances shall always be associated with a HumanSkeletonInterface
      with the same ID. This interface is particularly useful for RGBD cameras.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
HumanSkeletonProjectionInterface::HumanSkeletonProjectionInterface() : Interface()
{
  data_size = sizeof(HumanSkeletonProjectionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (HumanSkeletonProjectionInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_FLOAT, "horizontal_fov", 1, &data->horizontal_fov);
  add_fieldinfo(IFT_FLOAT, "vertical_fov", 1, &data->vertical_fov);
  add_fieldinfo(IFT_UINT32, "res_x", 1, &data->res_x);
  add_fieldinfo(IFT_UINT32, "res_y", 1, &data->res_y);
  add_fieldinfo(IFT_UINT16, "max_depth", 1, &data->max_depth);
  add_fieldinfo(IFT_FLOAT, "proj_com", 2, &data->proj_com);
  add_fieldinfo(IFT_FLOAT, "proj_head", 2, &data->proj_head);
  add_fieldinfo(IFT_FLOAT, "proj_neck", 2, &data->proj_neck);
  add_fieldinfo(IFT_FLOAT, "proj_torso", 2, &data->proj_torso);
  add_fieldinfo(IFT_FLOAT, "proj_waist", 2, &data->proj_waist);
  add_fieldinfo(IFT_FLOAT, "proj_left_collar", 2, &data->proj_left_collar);
  add_fieldinfo(IFT_FLOAT, "proj_left_shoulder", 2, &data->proj_left_shoulder);
  add_fieldinfo(IFT_FLOAT, "proj_left_elbow", 2, &data->proj_left_elbow);
  add_fieldinfo(IFT_FLOAT, "proj_left_wrist", 2, &data->proj_left_wrist);
  add_fieldinfo(IFT_FLOAT, "proj_left_hand", 2, &data->proj_left_hand);
  add_fieldinfo(IFT_FLOAT, "proj_left_fingertip", 2, &data->proj_left_fingertip);
  add_fieldinfo(IFT_FLOAT, "proj_right_collar", 2, &data->proj_right_collar);
  add_fieldinfo(IFT_FLOAT, "proj_right_shoulder", 2, &data->proj_right_shoulder);
  add_fieldinfo(IFT_FLOAT, "proj_right_elbow", 2, &data->proj_right_elbow);
  add_fieldinfo(IFT_FLOAT, "proj_right_wrist", 2, &data->proj_right_wrist);
  add_fieldinfo(IFT_FLOAT, "proj_right_hand", 2, &data->proj_right_hand);
  add_fieldinfo(IFT_FLOAT, "proj_right_fingertip", 2, &data->proj_right_fingertip);
  add_fieldinfo(IFT_FLOAT, "proj_left_hip", 2, &data->proj_left_hip);
  add_fieldinfo(IFT_FLOAT, "proj_left_knee", 2, &data->proj_left_knee);
  add_fieldinfo(IFT_FLOAT, "proj_left_ankle", 2, &data->proj_left_ankle);
  add_fieldinfo(IFT_FLOAT, "proj_left_foot", 2, &data->proj_left_foot);
  add_fieldinfo(IFT_FLOAT, "proj_right_hip", 2, &data->proj_right_hip);
  add_fieldinfo(IFT_FLOAT, "proj_right_knee", 2, &data->proj_right_knee);
  add_fieldinfo(IFT_FLOAT, "proj_right_ankle", 2, &data->proj_right_ankle);
  add_fieldinfo(IFT_FLOAT, "proj_right_foot", 2, &data->proj_right_foot);
  unsigned char tmp_hash[] = {0x71, 0xb2, 0x40, 0x3e, 0xa, 0x85, 0xd5, 0xcc, 0x77, 0xeb, 0xf2, 0xf1, 0xa9, 0x9c, 0xec, 0xf3};
  set_hash(tmp_hash);
}

/** Destructor */
HumanSkeletonProjectionInterface::~HumanSkeletonProjectionInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get horizontal_fov value.
 * Opening angle in radians.
 * @return horizontal_fov value
 */
float
HumanSkeletonProjectionInterface::horizontal_fov() const
{
  return data->horizontal_fov;
}

/** Get maximum length of horizontal_fov value.
 * @return length of horizontal_fov value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_horizontal_fov() const
{
  return 1;
}

/** Set horizontal_fov value.
 * Opening angle in radians.
 * @param new_horizontal_fov new horizontal_fov value
 */
void
HumanSkeletonProjectionInterface::set_horizontal_fov(const float new_horizontal_fov)
{
  data->horizontal_fov = new_horizontal_fov;
  data_changed = true;
}

/** Get vertical_fov value.
 * Opening angle in radians.
 * @return vertical_fov value
 */
float
HumanSkeletonProjectionInterface::vertical_fov() const
{
  return data->vertical_fov;
}

/** Get maximum length of vertical_fov value.
 * @return length of vertical_fov value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_vertical_fov() const
{
  return 1;
}

/** Set vertical_fov value.
 * Opening angle in radians.
 * @param new_vertical_fov new vertical_fov value
 */
void
HumanSkeletonProjectionInterface::set_vertical_fov(const float new_vertical_fov)
{
  data->vertical_fov = new_vertical_fov;
  data_changed = true;
}

/** Get res_x value.
 * X resolution (number of columns in frame).
 * @return res_x value
 */
uint32_t
HumanSkeletonProjectionInterface::res_x() const
{
  return data->res_x;
}

/** Get maximum length of res_x value.
 * @return length of res_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_res_x() const
{
  return 1;
}

/** Set res_x value.
 * X resolution (number of columns in frame).
 * @param new_res_x new res_x value
 */
void
HumanSkeletonProjectionInterface::set_res_x(const uint32_t new_res_x)
{
  data->res_x = new_res_x;
  data_changed = true;
}

/** Get res_y value.
 * Y resolution (number of rows in frame).
 * @return res_y value
 */
uint32_t
HumanSkeletonProjectionInterface::res_y() const
{
  return data->res_y;
}

/** Get maximum length of res_y value.
 * @return length of res_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_res_y() const
{
  return 1;
}

/** Set res_y value.
 * Y resolution (number of rows in frame).
 * @param new_res_y new res_y value
 */
void
HumanSkeletonProjectionInterface::set_res_y(const uint32_t new_res_y)
{
  data->res_y = new_res_y;
  data_changed = true;
}

/** Get max_depth value.
 * Maximum depth value.
 * @return max_depth value
 */
uint16_t
HumanSkeletonProjectionInterface::max_depth() const
{
  return data->max_depth;
}

/** Get maximum length of max_depth value.
 * @return length of max_depth value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_max_depth() const
{
  return 1;
}

/** Set max_depth value.
 * Maximum depth value.
 * @param new_max_depth new max_depth value
 */
void
HumanSkeletonProjectionInterface::set_max_depth(const uint16_t new_max_depth)
{
  data->max_depth = new_max_depth;
  data_changed = true;
}

/** Get proj_com value.
 * Center of mass.
 * @return proj_com value
 */
float *
HumanSkeletonProjectionInterface::proj_com() const
{
  return data->proj_com;
}

/** Get proj_com value at given index.
 * Center of mass.
 * @param index index of value
 * @return proj_com value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_com(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_com[index];
}

/** Get maximum length of proj_com value.
 * @return length of proj_com value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_com() const
{
  return 2;
}

/** Set proj_com value.
 * Center of mass.
 * @param new_proj_com new proj_com value
 */
void
HumanSkeletonProjectionInterface::set_proj_com(const float * new_proj_com)
{
  memcpy(data->proj_com, new_proj_com, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_com value at given index.
 * Center of mass.
 * @param new_proj_com new proj_com value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_com(unsigned int index, const float new_proj_com)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_com[index] = new_proj_com;
  data_changed = true;
}
/** Get proj_head value.
 * Head position vector.
 * @return proj_head value
 */
float *
HumanSkeletonProjectionInterface::proj_head() const
{
  return data->proj_head;
}

/** Get proj_head value at given index.
 * Head position vector.
 * @param index index of value
 * @return proj_head value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_head(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_head[index];
}

/** Get maximum length of proj_head value.
 * @return length of proj_head value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_head() const
{
  return 2;
}

/** Set proj_head value.
 * Head position vector.
 * @param new_proj_head new proj_head value
 */
void
HumanSkeletonProjectionInterface::set_proj_head(const float * new_proj_head)
{
  memcpy(data->proj_head, new_proj_head, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_head value at given index.
 * Head position vector.
 * @param new_proj_head new proj_head value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_head(unsigned int index, const float new_proj_head)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_head[index] = new_proj_head;
  data_changed = true;
}
/** Get proj_neck value.
 * Neck position vector.
 * @return proj_neck value
 */
float *
HumanSkeletonProjectionInterface::proj_neck() const
{
  return data->proj_neck;
}

/** Get proj_neck value at given index.
 * Neck position vector.
 * @param index index of value
 * @return proj_neck value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_neck(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_neck[index];
}

/** Get maximum length of proj_neck value.
 * @return length of proj_neck value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_neck() const
{
  return 2;
}

/** Set proj_neck value.
 * Neck position vector.
 * @param new_proj_neck new proj_neck value
 */
void
HumanSkeletonProjectionInterface::set_proj_neck(const float * new_proj_neck)
{
  memcpy(data->proj_neck, new_proj_neck, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_neck value at given index.
 * Neck position vector.
 * @param new_proj_neck new proj_neck value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_neck(unsigned int index, const float new_proj_neck)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_neck[index] = new_proj_neck;
  data_changed = true;
}
/** Get proj_torso value.
 * Torso position vector.
 * @return proj_torso value
 */
float *
HumanSkeletonProjectionInterface::proj_torso() const
{
  return data->proj_torso;
}

/** Get proj_torso value at given index.
 * Torso position vector.
 * @param index index of value
 * @return proj_torso value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_torso(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_torso[index];
}

/** Get maximum length of proj_torso value.
 * @return length of proj_torso value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_torso() const
{
  return 2;
}

/** Set proj_torso value.
 * Torso position vector.
 * @param new_proj_torso new proj_torso value
 */
void
HumanSkeletonProjectionInterface::set_proj_torso(const float * new_proj_torso)
{
  memcpy(data->proj_torso, new_proj_torso, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_torso value at given index.
 * Torso position vector.
 * @param new_proj_torso new proj_torso value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_torso(unsigned int index, const float new_proj_torso)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_torso[index] = new_proj_torso;
  data_changed = true;
}
/** Get proj_waist value.
 * Waist position vector.
 * @return proj_waist value
 */
float *
HumanSkeletonProjectionInterface::proj_waist() const
{
  return data->proj_waist;
}

/** Get proj_waist value at given index.
 * Waist position vector.
 * @param index index of value
 * @return proj_waist value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_waist(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_waist[index];
}

/** Get maximum length of proj_waist value.
 * @return length of proj_waist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_waist() const
{
  return 2;
}

/** Set proj_waist value.
 * Waist position vector.
 * @param new_proj_waist new proj_waist value
 */
void
HumanSkeletonProjectionInterface::set_proj_waist(const float * new_proj_waist)
{
  memcpy(data->proj_waist, new_proj_waist, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_waist value at given index.
 * Waist position vector.
 * @param new_proj_waist new proj_waist value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_waist(unsigned int index, const float new_proj_waist)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_waist[index] = new_proj_waist;
  data_changed = true;
}
/** Get proj_left_collar value.
 * 
      Left position vector.
 * @return proj_left_collar value
 */
float *
HumanSkeletonProjectionInterface::proj_left_collar() const
{
  return data->proj_left_collar;
}

/** Get proj_left_collar value at given index.
 * 
      Left position vector.
 * @param index index of value
 * @return proj_left_collar value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_left_collar(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_left_collar[index];
}

/** Get maximum length of proj_left_collar value.
 * @return length of proj_left_collar value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_left_collar() const
{
  return 2;
}

/** Set proj_left_collar value.
 * 
      Left position vector.
 * @param new_proj_left_collar new proj_left_collar value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_collar(const float * new_proj_left_collar)
{
  memcpy(data->proj_left_collar, new_proj_left_collar, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_left_collar value at given index.
 * 
      Left position vector.
 * @param new_proj_left_collar new proj_left_collar value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_collar(unsigned int index, const float new_proj_left_collar)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_left_collar[index] = new_proj_left_collar;
  data_changed = true;
}
/** Get proj_left_shoulder value.
 * 
      Left shoulder position vector.
 * @return proj_left_shoulder value
 */
float *
HumanSkeletonProjectionInterface::proj_left_shoulder() const
{
  return data->proj_left_shoulder;
}

/** Get proj_left_shoulder value at given index.
 * 
      Left shoulder position vector.
 * @param index index of value
 * @return proj_left_shoulder value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_left_shoulder(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_left_shoulder[index];
}

/** Get maximum length of proj_left_shoulder value.
 * @return length of proj_left_shoulder value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_left_shoulder() const
{
  return 2;
}

/** Set proj_left_shoulder value.
 * 
      Left shoulder position vector.
 * @param new_proj_left_shoulder new proj_left_shoulder value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_shoulder(const float * new_proj_left_shoulder)
{
  memcpy(data->proj_left_shoulder, new_proj_left_shoulder, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_left_shoulder value at given index.
 * 
      Left shoulder position vector.
 * @param new_proj_left_shoulder new proj_left_shoulder value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_shoulder(unsigned int index, const float new_proj_left_shoulder)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_left_shoulder[index] = new_proj_left_shoulder;
  data_changed = true;
}
/** Get proj_left_elbow value.
 * 
      Left elbow position vector.
 * @return proj_left_elbow value
 */
float *
HumanSkeletonProjectionInterface::proj_left_elbow() const
{
  return data->proj_left_elbow;
}

/** Get proj_left_elbow value at given index.
 * 
      Left elbow position vector.
 * @param index index of value
 * @return proj_left_elbow value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_left_elbow(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_left_elbow[index];
}

/** Get maximum length of proj_left_elbow value.
 * @return length of proj_left_elbow value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_left_elbow() const
{
  return 2;
}

/** Set proj_left_elbow value.
 * 
      Left elbow position vector.
 * @param new_proj_left_elbow new proj_left_elbow value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_elbow(const float * new_proj_left_elbow)
{
  memcpy(data->proj_left_elbow, new_proj_left_elbow, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_left_elbow value at given index.
 * 
      Left elbow position vector.
 * @param new_proj_left_elbow new proj_left_elbow value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_elbow(unsigned int index, const float new_proj_left_elbow)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_left_elbow[index] = new_proj_left_elbow;
  data_changed = true;
}
/** Get proj_left_wrist value.
 * 
      Left wrist position vector.
 * @return proj_left_wrist value
 */
float *
HumanSkeletonProjectionInterface::proj_left_wrist() const
{
  return data->proj_left_wrist;
}

/** Get proj_left_wrist value at given index.
 * 
      Left wrist position vector.
 * @param index index of value
 * @return proj_left_wrist value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_left_wrist(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_left_wrist[index];
}

/** Get maximum length of proj_left_wrist value.
 * @return length of proj_left_wrist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_left_wrist() const
{
  return 2;
}

/** Set proj_left_wrist value.
 * 
      Left wrist position vector.
 * @param new_proj_left_wrist new proj_left_wrist value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_wrist(const float * new_proj_left_wrist)
{
  memcpy(data->proj_left_wrist, new_proj_left_wrist, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_left_wrist value at given index.
 * 
      Left wrist position vector.
 * @param new_proj_left_wrist new proj_left_wrist value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_wrist(unsigned int index, const float new_proj_left_wrist)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_left_wrist[index] = new_proj_left_wrist;
  data_changed = true;
}
/** Get proj_left_hand value.
 * 
      Left hand position vector.
 * @return proj_left_hand value
 */
float *
HumanSkeletonProjectionInterface::proj_left_hand() const
{
  return data->proj_left_hand;
}

/** Get proj_left_hand value at given index.
 * 
      Left hand position vector.
 * @param index index of value
 * @return proj_left_hand value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_left_hand(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_left_hand[index];
}

/** Get maximum length of proj_left_hand value.
 * @return length of proj_left_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_left_hand() const
{
  return 2;
}

/** Set proj_left_hand value.
 * 
      Left hand position vector.
 * @param new_proj_left_hand new proj_left_hand value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_hand(const float * new_proj_left_hand)
{
  memcpy(data->proj_left_hand, new_proj_left_hand, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_left_hand value at given index.
 * 
      Left hand position vector.
 * @param new_proj_left_hand new proj_left_hand value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_hand(unsigned int index, const float new_proj_left_hand)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_left_hand[index] = new_proj_left_hand;
  data_changed = true;
}
/** Get proj_left_fingertip value.
 * 
      Left fingertip position vector.
 * @return proj_left_fingertip value
 */
float *
HumanSkeletonProjectionInterface::proj_left_fingertip() const
{
  return data->proj_left_fingertip;
}

/** Get proj_left_fingertip value at given index.
 * 
      Left fingertip position vector.
 * @param index index of value
 * @return proj_left_fingertip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_left_fingertip(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_left_fingertip[index];
}

/** Get maximum length of proj_left_fingertip value.
 * @return length of proj_left_fingertip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_left_fingertip() const
{
  return 2;
}

/** Set proj_left_fingertip value.
 * 
      Left fingertip position vector.
 * @param new_proj_left_fingertip new proj_left_fingertip value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_fingertip(const float * new_proj_left_fingertip)
{
  memcpy(data->proj_left_fingertip, new_proj_left_fingertip, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_left_fingertip value at given index.
 * 
      Left fingertip position vector.
 * @param new_proj_left_fingertip new proj_left_fingertip value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_fingertip(unsigned int index, const float new_proj_left_fingertip)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_left_fingertip[index] = new_proj_left_fingertip;
  data_changed = true;
}
/** Get proj_right_collar value.
 * 
      Right collar position vector.
 * @return proj_right_collar value
 */
float *
HumanSkeletonProjectionInterface::proj_right_collar() const
{
  return data->proj_right_collar;
}

/** Get proj_right_collar value at given index.
 * 
      Right collar position vector.
 * @param index index of value
 * @return proj_right_collar value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_right_collar(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_right_collar[index];
}

/** Get maximum length of proj_right_collar value.
 * @return length of proj_right_collar value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_right_collar() const
{
  return 2;
}

/** Set proj_right_collar value.
 * 
      Right collar position vector.
 * @param new_proj_right_collar new proj_right_collar value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_collar(const float * new_proj_right_collar)
{
  memcpy(data->proj_right_collar, new_proj_right_collar, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_right_collar value at given index.
 * 
      Right collar position vector.
 * @param new_proj_right_collar new proj_right_collar value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_collar(unsigned int index, const float new_proj_right_collar)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_right_collar[index] = new_proj_right_collar;
  data_changed = true;
}
/** Get proj_right_shoulder value.
 * 
      Right shoulder position vector.
 * @return proj_right_shoulder value
 */
float *
HumanSkeletonProjectionInterface::proj_right_shoulder() const
{
  return data->proj_right_shoulder;
}

/** Get proj_right_shoulder value at given index.
 * 
      Right shoulder position vector.
 * @param index index of value
 * @return proj_right_shoulder value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_right_shoulder(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_right_shoulder[index];
}

/** Get maximum length of proj_right_shoulder value.
 * @return length of proj_right_shoulder value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_right_shoulder() const
{
  return 2;
}

/** Set proj_right_shoulder value.
 * 
      Right shoulder position vector.
 * @param new_proj_right_shoulder new proj_right_shoulder value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_shoulder(const float * new_proj_right_shoulder)
{
  memcpy(data->proj_right_shoulder, new_proj_right_shoulder, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_right_shoulder value at given index.
 * 
      Right shoulder position vector.
 * @param new_proj_right_shoulder new proj_right_shoulder value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_shoulder(unsigned int index, const float new_proj_right_shoulder)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_right_shoulder[index] = new_proj_right_shoulder;
  data_changed = true;
}
/** Get proj_right_elbow value.
 * 
      Right elbow position vector.
 * @return proj_right_elbow value
 */
float *
HumanSkeletonProjectionInterface::proj_right_elbow() const
{
  return data->proj_right_elbow;
}

/** Get proj_right_elbow value at given index.
 * 
      Right elbow position vector.
 * @param index index of value
 * @return proj_right_elbow value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_right_elbow(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_right_elbow[index];
}

/** Get maximum length of proj_right_elbow value.
 * @return length of proj_right_elbow value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_right_elbow() const
{
  return 2;
}

/** Set proj_right_elbow value.
 * 
      Right elbow position vector.
 * @param new_proj_right_elbow new proj_right_elbow value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_elbow(const float * new_proj_right_elbow)
{
  memcpy(data->proj_right_elbow, new_proj_right_elbow, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_right_elbow value at given index.
 * 
      Right elbow position vector.
 * @param new_proj_right_elbow new proj_right_elbow value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_elbow(unsigned int index, const float new_proj_right_elbow)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_right_elbow[index] = new_proj_right_elbow;
  data_changed = true;
}
/** Get proj_right_wrist value.
 * 
      Right wrist position vector.
 * @return proj_right_wrist value
 */
float *
HumanSkeletonProjectionInterface::proj_right_wrist() const
{
  return data->proj_right_wrist;
}

/** Get proj_right_wrist value at given index.
 * 
      Right wrist position vector.
 * @param index index of value
 * @return proj_right_wrist value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_right_wrist(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_right_wrist[index];
}

/** Get maximum length of proj_right_wrist value.
 * @return length of proj_right_wrist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_right_wrist() const
{
  return 2;
}

/** Set proj_right_wrist value.
 * 
      Right wrist position vector.
 * @param new_proj_right_wrist new proj_right_wrist value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_wrist(const float * new_proj_right_wrist)
{
  memcpy(data->proj_right_wrist, new_proj_right_wrist, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_right_wrist value at given index.
 * 
      Right wrist position vector.
 * @param new_proj_right_wrist new proj_right_wrist value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_wrist(unsigned int index, const float new_proj_right_wrist)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_right_wrist[index] = new_proj_right_wrist;
  data_changed = true;
}
/** Get proj_right_hand value.
 * 
      Right hand position vector.
 * @return proj_right_hand value
 */
float *
HumanSkeletonProjectionInterface::proj_right_hand() const
{
  return data->proj_right_hand;
}

/** Get proj_right_hand value at given index.
 * 
      Right hand position vector.
 * @param index index of value
 * @return proj_right_hand value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_right_hand(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_right_hand[index];
}

/** Get maximum length of proj_right_hand value.
 * @return length of proj_right_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_right_hand() const
{
  return 2;
}

/** Set proj_right_hand value.
 * 
      Right hand position vector.
 * @param new_proj_right_hand new proj_right_hand value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_hand(const float * new_proj_right_hand)
{
  memcpy(data->proj_right_hand, new_proj_right_hand, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_right_hand value at given index.
 * 
      Right hand position vector.
 * @param new_proj_right_hand new proj_right_hand value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_hand(unsigned int index, const float new_proj_right_hand)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_right_hand[index] = new_proj_right_hand;
  data_changed = true;
}
/** Get proj_right_fingertip value.
 * 
      Right fingertip position vector.
 * @return proj_right_fingertip value
 */
float *
HumanSkeletonProjectionInterface::proj_right_fingertip() const
{
  return data->proj_right_fingertip;
}

/** Get proj_right_fingertip value at given index.
 * 
      Right fingertip position vector.
 * @param index index of value
 * @return proj_right_fingertip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_right_fingertip(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_right_fingertip[index];
}

/** Get maximum length of proj_right_fingertip value.
 * @return length of proj_right_fingertip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_right_fingertip() const
{
  return 2;
}

/** Set proj_right_fingertip value.
 * 
      Right fingertip position vector.
 * @param new_proj_right_fingertip new proj_right_fingertip value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_fingertip(const float * new_proj_right_fingertip)
{
  memcpy(data->proj_right_fingertip, new_proj_right_fingertip, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_right_fingertip value at given index.
 * 
      Right fingertip position vector.
 * @param new_proj_right_fingertip new proj_right_fingertip value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_fingertip(unsigned int index, const float new_proj_right_fingertip)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_right_fingertip[index] = new_proj_right_fingertip;
  data_changed = true;
}
/** Get proj_left_hip value.
 * 
      Left hip position vector.
 * @return proj_left_hip value
 */
float *
HumanSkeletonProjectionInterface::proj_left_hip() const
{
  return data->proj_left_hip;
}

/** Get proj_left_hip value at given index.
 * 
      Left hip position vector.
 * @param index index of value
 * @return proj_left_hip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_left_hip(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_left_hip[index];
}

/** Get maximum length of proj_left_hip value.
 * @return length of proj_left_hip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_left_hip() const
{
  return 2;
}

/** Set proj_left_hip value.
 * 
      Left hip position vector.
 * @param new_proj_left_hip new proj_left_hip value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_hip(const float * new_proj_left_hip)
{
  memcpy(data->proj_left_hip, new_proj_left_hip, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_left_hip value at given index.
 * 
      Left hip position vector.
 * @param new_proj_left_hip new proj_left_hip value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_hip(unsigned int index, const float new_proj_left_hip)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_left_hip[index] = new_proj_left_hip;
  data_changed = true;
}
/** Get proj_left_knee value.
 * 
      Left knee position vector.
 * @return proj_left_knee value
 */
float *
HumanSkeletonProjectionInterface::proj_left_knee() const
{
  return data->proj_left_knee;
}

/** Get proj_left_knee value at given index.
 * 
      Left knee position vector.
 * @param index index of value
 * @return proj_left_knee value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_left_knee(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_left_knee[index];
}

/** Get maximum length of proj_left_knee value.
 * @return length of proj_left_knee value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_left_knee() const
{
  return 2;
}

/** Set proj_left_knee value.
 * 
      Left knee position vector.
 * @param new_proj_left_knee new proj_left_knee value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_knee(const float * new_proj_left_knee)
{
  memcpy(data->proj_left_knee, new_proj_left_knee, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_left_knee value at given index.
 * 
      Left knee position vector.
 * @param new_proj_left_knee new proj_left_knee value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_knee(unsigned int index, const float new_proj_left_knee)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_left_knee[index] = new_proj_left_knee;
  data_changed = true;
}
/** Get proj_left_ankle value.
 * 
      Left ankle position vector.
 * @return proj_left_ankle value
 */
float *
HumanSkeletonProjectionInterface::proj_left_ankle() const
{
  return data->proj_left_ankle;
}

/** Get proj_left_ankle value at given index.
 * 
      Left ankle position vector.
 * @param index index of value
 * @return proj_left_ankle value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_left_ankle(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_left_ankle[index];
}

/** Get maximum length of proj_left_ankle value.
 * @return length of proj_left_ankle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_left_ankle() const
{
  return 2;
}

/** Set proj_left_ankle value.
 * 
      Left ankle position vector.
 * @param new_proj_left_ankle new proj_left_ankle value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_ankle(const float * new_proj_left_ankle)
{
  memcpy(data->proj_left_ankle, new_proj_left_ankle, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_left_ankle value at given index.
 * 
      Left ankle position vector.
 * @param new_proj_left_ankle new proj_left_ankle value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_ankle(unsigned int index, const float new_proj_left_ankle)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_left_ankle[index] = new_proj_left_ankle;
  data_changed = true;
}
/** Get proj_left_foot value.
 * 
      Left foot position vector.
 * @return proj_left_foot value
 */
float *
HumanSkeletonProjectionInterface::proj_left_foot() const
{
  return data->proj_left_foot;
}

/** Get proj_left_foot value at given index.
 * 
      Left foot position vector.
 * @param index index of value
 * @return proj_left_foot value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_left_foot(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_left_foot[index];
}

/** Get maximum length of proj_left_foot value.
 * @return length of proj_left_foot value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_left_foot() const
{
  return 2;
}

/** Set proj_left_foot value.
 * 
      Left foot position vector.
 * @param new_proj_left_foot new proj_left_foot value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_foot(const float * new_proj_left_foot)
{
  memcpy(data->proj_left_foot, new_proj_left_foot, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_left_foot value at given index.
 * 
      Left foot position vector.
 * @param new_proj_left_foot new proj_left_foot value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_left_foot(unsigned int index, const float new_proj_left_foot)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_left_foot[index] = new_proj_left_foot;
  data_changed = true;
}
/** Get proj_right_hip value.
 * 
      Right hip position vector.
 * @return proj_right_hip value
 */
float *
HumanSkeletonProjectionInterface::proj_right_hip() const
{
  return data->proj_right_hip;
}

/** Get proj_right_hip value at given index.
 * 
      Right hip position vector.
 * @param index index of value
 * @return proj_right_hip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_right_hip(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_right_hip[index];
}

/** Get maximum length of proj_right_hip value.
 * @return length of proj_right_hip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_right_hip() const
{
  return 2;
}

/** Set proj_right_hip value.
 * 
      Right hip position vector.
 * @param new_proj_right_hip new proj_right_hip value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_hip(const float * new_proj_right_hip)
{
  memcpy(data->proj_right_hip, new_proj_right_hip, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_right_hip value at given index.
 * 
      Right hip position vector.
 * @param new_proj_right_hip new proj_right_hip value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_hip(unsigned int index, const float new_proj_right_hip)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_right_hip[index] = new_proj_right_hip;
  data_changed = true;
}
/** Get proj_right_knee value.
 * 
      Right knee position vector.
 * @return proj_right_knee value
 */
float *
HumanSkeletonProjectionInterface::proj_right_knee() const
{
  return data->proj_right_knee;
}

/** Get proj_right_knee value at given index.
 * 
      Right knee position vector.
 * @param index index of value
 * @return proj_right_knee value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_right_knee(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_right_knee[index];
}

/** Get maximum length of proj_right_knee value.
 * @return length of proj_right_knee value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_right_knee() const
{
  return 2;
}

/** Set proj_right_knee value.
 * 
      Right knee position vector.
 * @param new_proj_right_knee new proj_right_knee value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_knee(const float * new_proj_right_knee)
{
  memcpy(data->proj_right_knee, new_proj_right_knee, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_right_knee value at given index.
 * 
      Right knee position vector.
 * @param new_proj_right_knee new proj_right_knee value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_knee(unsigned int index, const float new_proj_right_knee)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_right_knee[index] = new_proj_right_knee;
  data_changed = true;
}
/** Get proj_right_ankle value.
 * 
      Right ankle position vector.
 * @return proj_right_ankle value
 */
float *
HumanSkeletonProjectionInterface::proj_right_ankle() const
{
  return data->proj_right_ankle;
}

/** Get proj_right_ankle value at given index.
 * 
      Right ankle position vector.
 * @param index index of value
 * @return proj_right_ankle value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_right_ankle(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_right_ankle[index];
}

/** Get maximum length of proj_right_ankle value.
 * @return length of proj_right_ankle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_right_ankle() const
{
  return 2;
}

/** Set proj_right_ankle value.
 * 
      Right ankle position vector.
 * @param new_proj_right_ankle new proj_right_ankle value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_ankle(const float * new_proj_right_ankle)
{
  memcpy(data->proj_right_ankle, new_proj_right_ankle, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_right_ankle value at given index.
 * 
      Right ankle position vector.
 * @param new_proj_right_ankle new proj_right_ankle value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_ankle(unsigned int index, const float new_proj_right_ankle)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_right_ankle[index] = new_proj_right_ankle;
  data_changed = true;
}
/** Get proj_right_foot value.
 * 
      Right foot position vector.
 * @return proj_right_foot value
 */
float *
HumanSkeletonProjectionInterface::proj_right_foot() const
{
  return data->proj_right_foot;
}

/** Get proj_right_foot value at given index.
 * 
      Right foot position vector.
 * @param index index of value
 * @return proj_right_foot value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonProjectionInterface::proj_right_foot(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->proj_right_foot[index];
}

/** Get maximum length of proj_right_foot value.
 * @return length of proj_right_foot value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonProjectionInterface::maxlenof_proj_right_foot() const
{
  return 2;
}

/** Set proj_right_foot value.
 * 
      Right foot position vector.
 * @param new_proj_right_foot new proj_right_foot value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_foot(const float * new_proj_right_foot)
{
  memcpy(data->proj_right_foot, new_proj_right_foot, sizeof(float) * 2);
  data_changed = true;
}

/** Set proj_right_foot value at given index.
 * 
      Right foot position vector.
 * @param new_proj_right_foot new proj_right_foot value
 * @param index index for of the value
 */
void
HumanSkeletonProjectionInterface::set_proj_right_foot(unsigned int index, const float new_proj_right_foot)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->proj_right_foot[index] = new_proj_right_foot;
  data_changed = true;
}
/* =========== message create =========== */
Message *
HumanSkeletonProjectionInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
HumanSkeletonProjectionInterface::copy_values(const Interface *other)
{
  const HumanSkeletonProjectionInterface *oi = dynamic_cast<const HumanSkeletonProjectionInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(HumanSkeletonProjectionInterface_data_t));
}

const char *
HumanSkeletonProjectionInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
HumanSkeletonProjectionInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(HumanSkeletonProjectionInterface)
/// @endcond


} // end namespace fawkes
