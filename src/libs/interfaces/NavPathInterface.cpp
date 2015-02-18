
/***************************************************************************
 *  NavPathInterface.cpp - Fawkes BlackBoard Interface - NavPathInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2014  Sebastian Reuter
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

#include <interfaces/NavPathInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class NavPathInterface <interfaces/NavPathInterface.h>
 * NavPathInterface Fawkes BlackBoard Interface.
 * 
      The navigator interface is used by the navigator to export information about
      the current status of the navigator and to define all messages by which the navigator
      can be instructed.

      There are three coordinate systems, the robot system which is a right-handed cartesian
      coordinate system with the robot in its origin, X axis pointing forward, Y pointing to
      the left and Z pointing upwards. The second coordinate system is the so-called
      navigator system. It is a coordinate system similar to the robot system, but the
      origin is defined on the initialization of the navigator. The last system is the
      odometry system. It is again a similar system, but the origin is reset from time
      to time and the robot's position in this system gives the odometry deltas.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
NavPathInterface::NavPathInterface() : Interface()
{
  data_size = sizeof(NavPathInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NavPathInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "path_node_1", 64, data->path_node_1);
  add_fieldinfo(IFT_STRING, "path_node_2", 64, data->path_node_2);
  add_fieldinfo(IFT_STRING, "path_node_3", 64, data->path_node_3);
  add_fieldinfo(IFT_STRING, "path_node_4", 64, data->path_node_4);
  add_fieldinfo(IFT_STRING, "path_node_5", 64, data->path_node_5);
  add_fieldinfo(IFT_STRING, "path_node_6", 64, data->path_node_6);
  add_fieldinfo(IFT_STRING, "path_node_7", 64, data->path_node_7);
  add_fieldinfo(IFT_STRING, "path_node_8", 64, data->path_node_8);
  add_fieldinfo(IFT_STRING, "path_node_9", 64, data->path_node_9);
  add_fieldinfo(IFT_STRING, "path_node_10", 64, data->path_node_10);
  add_fieldinfo(IFT_STRING, "path_node_11", 64, data->path_node_11);
  add_fieldinfo(IFT_STRING, "path_node_12", 64, data->path_node_12);
  add_fieldinfo(IFT_STRING, "path_node_13", 64, data->path_node_13);
  add_fieldinfo(IFT_STRING, "path_node_14", 64, data->path_node_14);
  add_fieldinfo(IFT_STRING, "path_node_15", 64, data->path_node_15);
  add_fieldinfo(IFT_STRING, "path_node_16", 64, data->path_node_16);
  add_fieldinfo(IFT_STRING, "path_node_17", 64, data->path_node_17);
  add_fieldinfo(IFT_STRING, "path_node_18", 64, data->path_node_18);
  add_fieldinfo(IFT_STRING, "path_node_19", 64, data->path_node_19);
  add_fieldinfo(IFT_STRING, "path_node_20", 64, data->path_node_20);
  add_fieldinfo(IFT_STRING, "path_node_21", 64, data->path_node_21);
  add_fieldinfo(IFT_STRING, "path_node_22", 64, data->path_node_22);
  add_fieldinfo(IFT_STRING, "path_node_23", 64, data->path_node_23);
  add_fieldinfo(IFT_STRING, "path_node_24", 64, data->path_node_24);
  add_fieldinfo(IFT_STRING, "path_node_25", 64, data->path_node_25);
  add_fieldinfo(IFT_STRING, "path_node_26", 64, data->path_node_26);
  add_fieldinfo(IFT_STRING, "path_node_27", 64, data->path_node_27);
  add_fieldinfo(IFT_STRING, "path_node_28", 64, data->path_node_28);
  add_fieldinfo(IFT_STRING, "path_node_29", 64, data->path_node_29);
  add_fieldinfo(IFT_STRING, "path_node_30", 64, data->path_node_30);
  add_fieldinfo(IFT_STRING, "path_node_31", 64, data->path_node_31);
  add_fieldinfo(IFT_STRING, "path_node_32", 64, data->path_node_32);
  add_fieldinfo(IFT_STRING, "path_node_33", 64, data->path_node_33);
  add_fieldinfo(IFT_STRING, "path_node_34", 64, data->path_node_34);
  add_fieldinfo(IFT_STRING, "path_node_35", 64, data->path_node_35);
  add_fieldinfo(IFT_STRING, "path_node_36", 64, data->path_node_36);
  add_fieldinfo(IFT_STRING, "path_node_37", 64, data->path_node_37);
  add_fieldinfo(IFT_STRING, "path_node_38", 64, data->path_node_38);
  add_fieldinfo(IFT_STRING, "path_node_39", 64, data->path_node_39);
  add_fieldinfo(IFT_STRING, "path_node_40", 64, data->path_node_40);
  add_fieldinfo(IFT_UINT32, "path_length", 1, &data->path_length);
  unsigned char tmp_hash[] = {0x9d, 0xe, 0xdb, 0x61, 0x65, 0x94, 0x3d, 0x7a, 0x87, 0x95, 0x8f, 0x85, 0x87, 0xa4, 0x5f, 0x61};
  set_hash(tmp_hash);
}

/** Destructor */
NavPathInterface::~NavPathInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get path_node_1 value.
 * Path Node
 * @return path_node_1 value
 */
char *
NavPathInterface::path_node_1() const
{
  return data->path_node_1;
}

/** Get maximum length of path_node_1 value.
 * @return length of path_node_1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_1() const
{
  return 64;
}

/** Set path_node_1 value.
 * Path Node
 * @param new_path_node_1 new path_node_1 value
 */
void
NavPathInterface::set_path_node_1(const char * new_path_node_1)
{
  strncpy(data->path_node_1, new_path_node_1, sizeof(data->path_node_1));
  data_changed = true;
}

/** Get path_node_2 value.
 * Path Node
 * @return path_node_2 value
 */
char *
NavPathInterface::path_node_2() const
{
  return data->path_node_2;
}

/** Get maximum length of path_node_2 value.
 * @return length of path_node_2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_2() const
{
  return 64;
}

/** Set path_node_2 value.
 * Path Node
 * @param new_path_node_2 new path_node_2 value
 */
void
NavPathInterface::set_path_node_2(const char * new_path_node_2)
{
  strncpy(data->path_node_2, new_path_node_2, sizeof(data->path_node_2));
  data_changed = true;
}

/** Get path_node_3 value.
 * Path Node
 * @return path_node_3 value
 */
char *
NavPathInterface::path_node_3() const
{
  return data->path_node_3;
}

/** Get maximum length of path_node_3 value.
 * @return length of path_node_3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_3() const
{
  return 64;
}

/** Set path_node_3 value.
 * Path Node
 * @param new_path_node_3 new path_node_3 value
 */
void
NavPathInterface::set_path_node_3(const char * new_path_node_3)
{
  strncpy(data->path_node_3, new_path_node_3, sizeof(data->path_node_3));
  data_changed = true;
}

/** Get path_node_4 value.
 * Path Node
 * @return path_node_4 value
 */
char *
NavPathInterface::path_node_4() const
{
  return data->path_node_4;
}

/** Get maximum length of path_node_4 value.
 * @return length of path_node_4 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_4() const
{
  return 64;
}

/** Set path_node_4 value.
 * Path Node
 * @param new_path_node_4 new path_node_4 value
 */
void
NavPathInterface::set_path_node_4(const char * new_path_node_4)
{
  strncpy(data->path_node_4, new_path_node_4, sizeof(data->path_node_4));
  data_changed = true;
}

/** Get path_node_5 value.
 * Path Node
 * @return path_node_5 value
 */
char *
NavPathInterface::path_node_5() const
{
  return data->path_node_5;
}

/** Get maximum length of path_node_5 value.
 * @return length of path_node_5 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_5() const
{
  return 64;
}

/** Set path_node_5 value.
 * Path Node
 * @param new_path_node_5 new path_node_5 value
 */
void
NavPathInterface::set_path_node_5(const char * new_path_node_5)
{
  strncpy(data->path_node_5, new_path_node_5, sizeof(data->path_node_5));
  data_changed = true;
}

/** Get path_node_6 value.
 * Path Node
 * @return path_node_6 value
 */
char *
NavPathInterface::path_node_6() const
{
  return data->path_node_6;
}

/** Get maximum length of path_node_6 value.
 * @return length of path_node_6 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_6() const
{
  return 64;
}

/** Set path_node_6 value.
 * Path Node
 * @param new_path_node_6 new path_node_6 value
 */
void
NavPathInterface::set_path_node_6(const char * new_path_node_6)
{
  strncpy(data->path_node_6, new_path_node_6, sizeof(data->path_node_6));
  data_changed = true;
}

/** Get path_node_7 value.
 * Path Node
 * @return path_node_7 value
 */
char *
NavPathInterface::path_node_7() const
{
  return data->path_node_7;
}

/** Get maximum length of path_node_7 value.
 * @return length of path_node_7 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_7() const
{
  return 64;
}

/** Set path_node_7 value.
 * Path Node
 * @param new_path_node_7 new path_node_7 value
 */
void
NavPathInterface::set_path_node_7(const char * new_path_node_7)
{
  strncpy(data->path_node_7, new_path_node_7, sizeof(data->path_node_7));
  data_changed = true;
}

/** Get path_node_8 value.
 * Path Node
 * @return path_node_8 value
 */
char *
NavPathInterface::path_node_8() const
{
  return data->path_node_8;
}

/** Get maximum length of path_node_8 value.
 * @return length of path_node_8 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_8() const
{
  return 64;
}

/** Set path_node_8 value.
 * Path Node
 * @param new_path_node_8 new path_node_8 value
 */
void
NavPathInterface::set_path_node_8(const char * new_path_node_8)
{
  strncpy(data->path_node_8, new_path_node_8, sizeof(data->path_node_8));
  data_changed = true;
}

/** Get path_node_9 value.
 * Path Node
 * @return path_node_9 value
 */
char *
NavPathInterface::path_node_9() const
{
  return data->path_node_9;
}

/** Get maximum length of path_node_9 value.
 * @return length of path_node_9 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_9() const
{
  return 64;
}

/** Set path_node_9 value.
 * Path Node
 * @param new_path_node_9 new path_node_9 value
 */
void
NavPathInterface::set_path_node_9(const char * new_path_node_9)
{
  strncpy(data->path_node_9, new_path_node_9, sizeof(data->path_node_9));
  data_changed = true;
}

/** Get path_node_10 value.
 * Path Node
 * @return path_node_10 value
 */
char *
NavPathInterface::path_node_10() const
{
  return data->path_node_10;
}

/** Get maximum length of path_node_10 value.
 * @return length of path_node_10 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_10() const
{
  return 64;
}

/** Set path_node_10 value.
 * Path Node
 * @param new_path_node_10 new path_node_10 value
 */
void
NavPathInterface::set_path_node_10(const char * new_path_node_10)
{
  strncpy(data->path_node_10, new_path_node_10, sizeof(data->path_node_10));
  data_changed = true;
}

/** Get path_node_11 value.
 * Path Node
 * @return path_node_11 value
 */
char *
NavPathInterface::path_node_11() const
{
  return data->path_node_11;
}

/** Get maximum length of path_node_11 value.
 * @return length of path_node_11 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_11() const
{
  return 64;
}

/** Set path_node_11 value.
 * Path Node
 * @param new_path_node_11 new path_node_11 value
 */
void
NavPathInterface::set_path_node_11(const char * new_path_node_11)
{
  strncpy(data->path_node_11, new_path_node_11, sizeof(data->path_node_11));
  data_changed = true;
}

/** Get path_node_12 value.
 * Path Node
 * @return path_node_12 value
 */
char *
NavPathInterface::path_node_12() const
{
  return data->path_node_12;
}

/** Get maximum length of path_node_12 value.
 * @return length of path_node_12 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_12() const
{
  return 64;
}

/** Set path_node_12 value.
 * Path Node
 * @param new_path_node_12 new path_node_12 value
 */
void
NavPathInterface::set_path_node_12(const char * new_path_node_12)
{
  strncpy(data->path_node_12, new_path_node_12, sizeof(data->path_node_12));
  data_changed = true;
}

/** Get path_node_13 value.
 * Path Node
 * @return path_node_13 value
 */
char *
NavPathInterface::path_node_13() const
{
  return data->path_node_13;
}

/** Get maximum length of path_node_13 value.
 * @return length of path_node_13 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_13() const
{
  return 64;
}

/** Set path_node_13 value.
 * Path Node
 * @param new_path_node_13 new path_node_13 value
 */
void
NavPathInterface::set_path_node_13(const char * new_path_node_13)
{
  strncpy(data->path_node_13, new_path_node_13, sizeof(data->path_node_13));
  data_changed = true;
}

/** Get path_node_14 value.
 * Path Node
 * @return path_node_14 value
 */
char *
NavPathInterface::path_node_14() const
{
  return data->path_node_14;
}

/** Get maximum length of path_node_14 value.
 * @return length of path_node_14 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_14() const
{
  return 64;
}

/** Set path_node_14 value.
 * Path Node
 * @param new_path_node_14 new path_node_14 value
 */
void
NavPathInterface::set_path_node_14(const char * new_path_node_14)
{
  strncpy(data->path_node_14, new_path_node_14, sizeof(data->path_node_14));
  data_changed = true;
}

/** Get path_node_15 value.
 * Path Node
 * @return path_node_15 value
 */
char *
NavPathInterface::path_node_15() const
{
  return data->path_node_15;
}

/** Get maximum length of path_node_15 value.
 * @return length of path_node_15 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_15() const
{
  return 64;
}

/** Set path_node_15 value.
 * Path Node
 * @param new_path_node_15 new path_node_15 value
 */
void
NavPathInterface::set_path_node_15(const char * new_path_node_15)
{
  strncpy(data->path_node_15, new_path_node_15, sizeof(data->path_node_15));
  data_changed = true;
}

/** Get path_node_16 value.
 * Path Node
 * @return path_node_16 value
 */
char *
NavPathInterface::path_node_16() const
{
  return data->path_node_16;
}

/** Get maximum length of path_node_16 value.
 * @return length of path_node_16 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_16() const
{
  return 64;
}

/** Set path_node_16 value.
 * Path Node
 * @param new_path_node_16 new path_node_16 value
 */
void
NavPathInterface::set_path_node_16(const char * new_path_node_16)
{
  strncpy(data->path_node_16, new_path_node_16, sizeof(data->path_node_16));
  data_changed = true;
}

/** Get path_node_17 value.
 * Path Node
 * @return path_node_17 value
 */
char *
NavPathInterface::path_node_17() const
{
  return data->path_node_17;
}

/** Get maximum length of path_node_17 value.
 * @return length of path_node_17 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_17() const
{
  return 64;
}

/** Set path_node_17 value.
 * Path Node
 * @param new_path_node_17 new path_node_17 value
 */
void
NavPathInterface::set_path_node_17(const char * new_path_node_17)
{
  strncpy(data->path_node_17, new_path_node_17, sizeof(data->path_node_17));
  data_changed = true;
}

/** Get path_node_18 value.
 * Path Node
 * @return path_node_18 value
 */
char *
NavPathInterface::path_node_18() const
{
  return data->path_node_18;
}

/** Get maximum length of path_node_18 value.
 * @return length of path_node_18 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_18() const
{
  return 64;
}

/** Set path_node_18 value.
 * Path Node
 * @param new_path_node_18 new path_node_18 value
 */
void
NavPathInterface::set_path_node_18(const char * new_path_node_18)
{
  strncpy(data->path_node_18, new_path_node_18, sizeof(data->path_node_18));
  data_changed = true;
}

/** Get path_node_19 value.
 * Path Node
 * @return path_node_19 value
 */
char *
NavPathInterface::path_node_19() const
{
  return data->path_node_19;
}

/** Get maximum length of path_node_19 value.
 * @return length of path_node_19 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_19() const
{
  return 64;
}

/** Set path_node_19 value.
 * Path Node
 * @param new_path_node_19 new path_node_19 value
 */
void
NavPathInterface::set_path_node_19(const char * new_path_node_19)
{
  strncpy(data->path_node_19, new_path_node_19, sizeof(data->path_node_19));
  data_changed = true;
}

/** Get path_node_20 value.
 * Path Node
 * @return path_node_20 value
 */
char *
NavPathInterface::path_node_20() const
{
  return data->path_node_20;
}

/** Get maximum length of path_node_20 value.
 * @return length of path_node_20 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_20() const
{
  return 64;
}

/** Set path_node_20 value.
 * Path Node
 * @param new_path_node_20 new path_node_20 value
 */
void
NavPathInterface::set_path_node_20(const char * new_path_node_20)
{
  strncpy(data->path_node_20, new_path_node_20, sizeof(data->path_node_20));
  data_changed = true;
}

/** Get path_node_21 value.
 * Path Node
 * @return path_node_21 value
 */
char *
NavPathInterface::path_node_21() const
{
  return data->path_node_21;
}

/** Get maximum length of path_node_21 value.
 * @return length of path_node_21 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_21() const
{
  return 64;
}

/** Set path_node_21 value.
 * Path Node
 * @param new_path_node_21 new path_node_21 value
 */
void
NavPathInterface::set_path_node_21(const char * new_path_node_21)
{
  strncpy(data->path_node_21, new_path_node_21, sizeof(data->path_node_21));
  data_changed = true;
}

/** Get path_node_22 value.
 * Path Node
 * @return path_node_22 value
 */
char *
NavPathInterface::path_node_22() const
{
  return data->path_node_22;
}

/** Get maximum length of path_node_22 value.
 * @return length of path_node_22 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_22() const
{
  return 64;
}

/** Set path_node_22 value.
 * Path Node
 * @param new_path_node_22 new path_node_22 value
 */
void
NavPathInterface::set_path_node_22(const char * new_path_node_22)
{
  strncpy(data->path_node_22, new_path_node_22, sizeof(data->path_node_22));
  data_changed = true;
}

/** Get path_node_23 value.
 * Path Node
 * @return path_node_23 value
 */
char *
NavPathInterface::path_node_23() const
{
  return data->path_node_23;
}

/** Get maximum length of path_node_23 value.
 * @return length of path_node_23 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_23() const
{
  return 64;
}

/** Set path_node_23 value.
 * Path Node
 * @param new_path_node_23 new path_node_23 value
 */
void
NavPathInterface::set_path_node_23(const char * new_path_node_23)
{
  strncpy(data->path_node_23, new_path_node_23, sizeof(data->path_node_23));
  data_changed = true;
}

/** Get path_node_24 value.
 * Path Node
 * @return path_node_24 value
 */
char *
NavPathInterface::path_node_24() const
{
  return data->path_node_24;
}

/** Get maximum length of path_node_24 value.
 * @return length of path_node_24 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_24() const
{
  return 64;
}

/** Set path_node_24 value.
 * Path Node
 * @param new_path_node_24 new path_node_24 value
 */
void
NavPathInterface::set_path_node_24(const char * new_path_node_24)
{
  strncpy(data->path_node_24, new_path_node_24, sizeof(data->path_node_24));
  data_changed = true;
}

/** Get path_node_25 value.
 * Path Node
 * @return path_node_25 value
 */
char *
NavPathInterface::path_node_25() const
{
  return data->path_node_25;
}

/** Get maximum length of path_node_25 value.
 * @return length of path_node_25 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_25() const
{
  return 64;
}

/** Set path_node_25 value.
 * Path Node
 * @param new_path_node_25 new path_node_25 value
 */
void
NavPathInterface::set_path_node_25(const char * new_path_node_25)
{
  strncpy(data->path_node_25, new_path_node_25, sizeof(data->path_node_25));
  data_changed = true;
}

/** Get path_node_26 value.
 * Path Node
 * @return path_node_26 value
 */
char *
NavPathInterface::path_node_26() const
{
  return data->path_node_26;
}

/** Get maximum length of path_node_26 value.
 * @return length of path_node_26 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_26() const
{
  return 64;
}

/** Set path_node_26 value.
 * Path Node
 * @param new_path_node_26 new path_node_26 value
 */
void
NavPathInterface::set_path_node_26(const char * new_path_node_26)
{
  strncpy(data->path_node_26, new_path_node_26, sizeof(data->path_node_26));
  data_changed = true;
}

/** Get path_node_27 value.
 * Path Node
 * @return path_node_27 value
 */
char *
NavPathInterface::path_node_27() const
{
  return data->path_node_27;
}

/** Get maximum length of path_node_27 value.
 * @return length of path_node_27 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_27() const
{
  return 64;
}

/** Set path_node_27 value.
 * Path Node
 * @param new_path_node_27 new path_node_27 value
 */
void
NavPathInterface::set_path_node_27(const char * new_path_node_27)
{
  strncpy(data->path_node_27, new_path_node_27, sizeof(data->path_node_27));
  data_changed = true;
}

/** Get path_node_28 value.
 * Path Node
 * @return path_node_28 value
 */
char *
NavPathInterface::path_node_28() const
{
  return data->path_node_28;
}

/** Get maximum length of path_node_28 value.
 * @return length of path_node_28 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_28() const
{
  return 64;
}

/** Set path_node_28 value.
 * Path Node
 * @param new_path_node_28 new path_node_28 value
 */
void
NavPathInterface::set_path_node_28(const char * new_path_node_28)
{
  strncpy(data->path_node_28, new_path_node_28, sizeof(data->path_node_28));
  data_changed = true;
}

/** Get path_node_29 value.
 * Path Node
 * @return path_node_29 value
 */
char *
NavPathInterface::path_node_29() const
{
  return data->path_node_29;
}

/** Get maximum length of path_node_29 value.
 * @return length of path_node_29 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_29() const
{
  return 64;
}

/** Set path_node_29 value.
 * Path Node
 * @param new_path_node_29 new path_node_29 value
 */
void
NavPathInterface::set_path_node_29(const char * new_path_node_29)
{
  strncpy(data->path_node_29, new_path_node_29, sizeof(data->path_node_29));
  data_changed = true;
}

/** Get path_node_30 value.
 * Path Node
 * @return path_node_30 value
 */
char *
NavPathInterface::path_node_30() const
{
  return data->path_node_30;
}

/** Get maximum length of path_node_30 value.
 * @return length of path_node_30 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_30() const
{
  return 64;
}

/** Set path_node_30 value.
 * Path Node
 * @param new_path_node_30 new path_node_30 value
 */
void
NavPathInterface::set_path_node_30(const char * new_path_node_30)
{
  strncpy(data->path_node_30, new_path_node_30, sizeof(data->path_node_30));
  data_changed = true;
}

/** Get path_node_31 value.
 * Path Node
 * @return path_node_31 value
 */
char *
NavPathInterface::path_node_31() const
{
  return data->path_node_31;
}

/** Get maximum length of path_node_31 value.
 * @return length of path_node_31 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_31() const
{
  return 64;
}

/** Set path_node_31 value.
 * Path Node
 * @param new_path_node_31 new path_node_31 value
 */
void
NavPathInterface::set_path_node_31(const char * new_path_node_31)
{
  strncpy(data->path_node_31, new_path_node_31, sizeof(data->path_node_31));
  data_changed = true;
}

/** Get path_node_32 value.
 * Path Node
 * @return path_node_32 value
 */
char *
NavPathInterface::path_node_32() const
{
  return data->path_node_32;
}

/** Get maximum length of path_node_32 value.
 * @return length of path_node_32 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_32() const
{
  return 64;
}

/** Set path_node_32 value.
 * Path Node
 * @param new_path_node_32 new path_node_32 value
 */
void
NavPathInterface::set_path_node_32(const char * new_path_node_32)
{
  strncpy(data->path_node_32, new_path_node_32, sizeof(data->path_node_32));
  data_changed = true;
}

/** Get path_node_33 value.
 * Path Node
 * @return path_node_33 value
 */
char *
NavPathInterface::path_node_33() const
{
  return data->path_node_33;
}

/** Get maximum length of path_node_33 value.
 * @return length of path_node_33 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_33() const
{
  return 64;
}

/** Set path_node_33 value.
 * Path Node
 * @param new_path_node_33 new path_node_33 value
 */
void
NavPathInterface::set_path_node_33(const char * new_path_node_33)
{
  strncpy(data->path_node_33, new_path_node_33, sizeof(data->path_node_33));
  data_changed = true;
}

/** Get path_node_34 value.
 * Path Node
 * @return path_node_34 value
 */
char *
NavPathInterface::path_node_34() const
{
  return data->path_node_34;
}

/** Get maximum length of path_node_34 value.
 * @return length of path_node_34 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_34() const
{
  return 64;
}

/** Set path_node_34 value.
 * Path Node
 * @param new_path_node_34 new path_node_34 value
 */
void
NavPathInterface::set_path_node_34(const char * new_path_node_34)
{
  strncpy(data->path_node_34, new_path_node_34, sizeof(data->path_node_34));
  data_changed = true;
}

/** Get path_node_35 value.
 * Path Node
 * @return path_node_35 value
 */
char *
NavPathInterface::path_node_35() const
{
  return data->path_node_35;
}

/** Get maximum length of path_node_35 value.
 * @return length of path_node_35 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_35() const
{
  return 64;
}

/** Set path_node_35 value.
 * Path Node
 * @param new_path_node_35 new path_node_35 value
 */
void
NavPathInterface::set_path_node_35(const char * new_path_node_35)
{
  strncpy(data->path_node_35, new_path_node_35, sizeof(data->path_node_35));
  data_changed = true;
}

/** Get path_node_36 value.
 * Path Node
 * @return path_node_36 value
 */
char *
NavPathInterface::path_node_36() const
{
  return data->path_node_36;
}

/** Get maximum length of path_node_36 value.
 * @return length of path_node_36 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_36() const
{
  return 64;
}

/** Set path_node_36 value.
 * Path Node
 * @param new_path_node_36 new path_node_36 value
 */
void
NavPathInterface::set_path_node_36(const char * new_path_node_36)
{
  strncpy(data->path_node_36, new_path_node_36, sizeof(data->path_node_36));
  data_changed = true;
}

/** Get path_node_37 value.
 * Path Node
 * @return path_node_37 value
 */
char *
NavPathInterface::path_node_37() const
{
  return data->path_node_37;
}

/** Get maximum length of path_node_37 value.
 * @return length of path_node_37 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_37() const
{
  return 64;
}

/** Set path_node_37 value.
 * Path Node
 * @param new_path_node_37 new path_node_37 value
 */
void
NavPathInterface::set_path_node_37(const char * new_path_node_37)
{
  strncpy(data->path_node_37, new_path_node_37, sizeof(data->path_node_37));
  data_changed = true;
}

/** Get path_node_38 value.
 * Path Node
 * @return path_node_38 value
 */
char *
NavPathInterface::path_node_38() const
{
  return data->path_node_38;
}

/** Get maximum length of path_node_38 value.
 * @return length of path_node_38 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_38() const
{
  return 64;
}

/** Set path_node_38 value.
 * Path Node
 * @param new_path_node_38 new path_node_38 value
 */
void
NavPathInterface::set_path_node_38(const char * new_path_node_38)
{
  strncpy(data->path_node_38, new_path_node_38, sizeof(data->path_node_38));
  data_changed = true;
}

/** Get path_node_39 value.
 * Path Node
 * @return path_node_39 value
 */
char *
NavPathInterface::path_node_39() const
{
  return data->path_node_39;
}

/** Get maximum length of path_node_39 value.
 * @return length of path_node_39 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_39() const
{
  return 64;
}

/** Set path_node_39 value.
 * Path Node
 * @param new_path_node_39 new path_node_39 value
 */
void
NavPathInterface::set_path_node_39(const char * new_path_node_39)
{
  strncpy(data->path_node_39, new_path_node_39, sizeof(data->path_node_39));
  data_changed = true;
}

/** Get path_node_40 value.
 * Path Node
 * @return path_node_40 value
 */
char *
NavPathInterface::path_node_40() const
{
  return data->path_node_40;
}

/** Get maximum length of path_node_40 value.
 * @return length of path_node_40 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_node_40() const
{
  return 64;
}

/** Set path_node_40 value.
 * Path Node
 * @param new_path_node_40 new path_node_40 value
 */
void
NavPathInterface::set_path_node_40(const char * new_path_node_40)
{
  strncpy(data->path_node_40, new_path_node_40, sizeof(data->path_node_40));
  data_changed = true;
}

/** Get path_length value.
 * Length of path
 * @return path_length value
 */
uint32_t
NavPathInterface::path_length() const
{
  return data->path_length;
}

/** Get maximum length of path_length value.
 * @return length of path_length value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavPathInterface::maxlenof_path_length() const
{
  return 1;
}

/** Set path_length value.
 * Length of path
 * @param new_path_length new path_length value
 */
void
NavPathInterface::set_path_length(const uint32_t new_path_length)
{
  data->path_length = new_path_length;
  data_changed = true;
}

/* =========== message create =========== */
Message *
NavPathInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
NavPathInterface::copy_values(const Interface *other)
{
  const NavPathInterface *oi = dynamic_cast<const NavPathInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(NavPathInterface_data_t));
}

const char *
NavPathInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
NavPathInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NavPathInterface)
/// @endcond


} // end namespace fawkes
