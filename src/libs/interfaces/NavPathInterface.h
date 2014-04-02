
/***************************************************************************
 *  NavPathInterface.h - Fawkes BlackBoard Interface - NavPathInterface
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

#ifndef __INTERFACES_NAVPATHINTERFACE_H_
#define __INTERFACES_NAVPATHINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class NavPathInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(NavPathInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char path_node_1[64]; /**< Path Node */
    char path_node_2[64]; /**< Path Node */
    char path_node_3[64]; /**< Path Node */
    char path_node_4[64]; /**< Path Node */
    char path_node_5[64]; /**< Path Node */
    char path_node_6[64]; /**< Path Node */
    char path_node_7[64]; /**< Path Node */
    char path_node_8[64]; /**< Path Node */
    char path_node_9[64]; /**< Path Node */
    char path_node_10[64]; /**< Path Node */
    char path_node_11[64]; /**< Path Node */
    char path_node_12[64]; /**< Path Node */
    char path_node_13[64]; /**< Path Node */
    char path_node_14[64]; /**< Path Node */
    char path_node_15[64]; /**< Path Node */
    char path_node_16[64]; /**< Path Node */
    char path_node_17[64]; /**< Path Node */
    char path_node_18[64]; /**< Path Node */
    char path_node_19[64]; /**< Path Node */
    char path_node_20[64]; /**< Path Node */
    char path_node_21[64]; /**< Path Node */
    char path_node_22[64]; /**< Path Node */
    char path_node_23[64]; /**< Path Node */
    char path_node_24[64]; /**< Path Node */
    char path_node_25[64]; /**< Path Node */
    char path_node_26[64]; /**< Path Node */
    char path_node_27[64]; /**< Path Node */
    char path_node_28[64]; /**< Path Node */
    char path_node_29[64]; /**< Path Node */
    char path_node_30[64]; /**< Path Node */
    char path_node_31[64]; /**< Path Node */
    char path_node_32[64]; /**< Path Node */
    char path_node_33[64]; /**< Path Node */
    char path_node_34[64]; /**< Path Node */
    char path_node_35[64]; /**< Path Node */
    char path_node_36[64]; /**< Path Node */
    char path_node_37[64]; /**< Path Node */
    char path_node_38[64]; /**< Path Node */
    char path_node_39[64]; /**< Path Node */
    char path_node_40[64]; /**< Path Node */
    uint32_t path_length; /**< Length of path */
  } NavPathInterface_data_t;
#pragma pack(pop)

  NavPathInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  NavPathInterface();
  ~NavPathInterface();

 public:
  /* Methods */
  char * path_node_1() const;
  void set_path_node_1(const char * new_path_node_1);
  size_t maxlenof_path_node_1() const;
  char * path_node_2() const;
  void set_path_node_2(const char * new_path_node_2);
  size_t maxlenof_path_node_2() const;
  char * path_node_3() const;
  void set_path_node_3(const char * new_path_node_3);
  size_t maxlenof_path_node_3() const;
  char * path_node_4() const;
  void set_path_node_4(const char * new_path_node_4);
  size_t maxlenof_path_node_4() const;
  char * path_node_5() const;
  void set_path_node_5(const char * new_path_node_5);
  size_t maxlenof_path_node_5() const;
  char * path_node_6() const;
  void set_path_node_6(const char * new_path_node_6);
  size_t maxlenof_path_node_6() const;
  char * path_node_7() const;
  void set_path_node_7(const char * new_path_node_7);
  size_t maxlenof_path_node_7() const;
  char * path_node_8() const;
  void set_path_node_8(const char * new_path_node_8);
  size_t maxlenof_path_node_8() const;
  char * path_node_9() const;
  void set_path_node_9(const char * new_path_node_9);
  size_t maxlenof_path_node_9() const;
  char * path_node_10() const;
  void set_path_node_10(const char * new_path_node_10);
  size_t maxlenof_path_node_10() const;
  char * path_node_11() const;
  void set_path_node_11(const char * new_path_node_11);
  size_t maxlenof_path_node_11() const;
  char * path_node_12() const;
  void set_path_node_12(const char * new_path_node_12);
  size_t maxlenof_path_node_12() const;
  char * path_node_13() const;
  void set_path_node_13(const char * new_path_node_13);
  size_t maxlenof_path_node_13() const;
  char * path_node_14() const;
  void set_path_node_14(const char * new_path_node_14);
  size_t maxlenof_path_node_14() const;
  char * path_node_15() const;
  void set_path_node_15(const char * new_path_node_15);
  size_t maxlenof_path_node_15() const;
  char * path_node_16() const;
  void set_path_node_16(const char * new_path_node_16);
  size_t maxlenof_path_node_16() const;
  char * path_node_17() const;
  void set_path_node_17(const char * new_path_node_17);
  size_t maxlenof_path_node_17() const;
  char * path_node_18() const;
  void set_path_node_18(const char * new_path_node_18);
  size_t maxlenof_path_node_18() const;
  char * path_node_19() const;
  void set_path_node_19(const char * new_path_node_19);
  size_t maxlenof_path_node_19() const;
  char * path_node_20() const;
  void set_path_node_20(const char * new_path_node_20);
  size_t maxlenof_path_node_20() const;
  char * path_node_21() const;
  void set_path_node_21(const char * new_path_node_21);
  size_t maxlenof_path_node_21() const;
  char * path_node_22() const;
  void set_path_node_22(const char * new_path_node_22);
  size_t maxlenof_path_node_22() const;
  char * path_node_23() const;
  void set_path_node_23(const char * new_path_node_23);
  size_t maxlenof_path_node_23() const;
  char * path_node_24() const;
  void set_path_node_24(const char * new_path_node_24);
  size_t maxlenof_path_node_24() const;
  char * path_node_25() const;
  void set_path_node_25(const char * new_path_node_25);
  size_t maxlenof_path_node_25() const;
  char * path_node_26() const;
  void set_path_node_26(const char * new_path_node_26);
  size_t maxlenof_path_node_26() const;
  char * path_node_27() const;
  void set_path_node_27(const char * new_path_node_27);
  size_t maxlenof_path_node_27() const;
  char * path_node_28() const;
  void set_path_node_28(const char * new_path_node_28);
  size_t maxlenof_path_node_28() const;
  char * path_node_29() const;
  void set_path_node_29(const char * new_path_node_29);
  size_t maxlenof_path_node_29() const;
  char * path_node_30() const;
  void set_path_node_30(const char * new_path_node_30);
  size_t maxlenof_path_node_30() const;
  char * path_node_31() const;
  void set_path_node_31(const char * new_path_node_31);
  size_t maxlenof_path_node_31() const;
  char * path_node_32() const;
  void set_path_node_32(const char * new_path_node_32);
  size_t maxlenof_path_node_32() const;
  char * path_node_33() const;
  void set_path_node_33(const char * new_path_node_33);
  size_t maxlenof_path_node_33() const;
  char * path_node_34() const;
  void set_path_node_34(const char * new_path_node_34);
  size_t maxlenof_path_node_34() const;
  char * path_node_35() const;
  void set_path_node_35(const char * new_path_node_35);
  size_t maxlenof_path_node_35() const;
  char * path_node_36() const;
  void set_path_node_36(const char * new_path_node_36);
  size_t maxlenof_path_node_36() const;
  char * path_node_37() const;
  void set_path_node_37(const char * new_path_node_37);
  size_t maxlenof_path_node_37() const;
  char * path_node_38() const;
  void set_path_node_38(const char * new_path_node_38);
  size_t maxlenof_path_node_38() const;
  char * path_node_39() const;
  void set_path_node_39(const char * new_path_node_39);
  size_t maxlenof_path_node_39() const;
  char * path_node_40() const;
  void set_path_node_40(const char * new_path_node_40);
  size_t maxlenof_path_node_40() const;
  uint32_t path_length() const;
  void set_path_length(const uint32_t new_path_length);
  size_t maxlenof_path_length() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
