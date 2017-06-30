
/***************************************************************************
 *  sick_tim55x_common_aqt.h - Super class of TiM55x drivers
 *
 *  Created: Sun Jun 15 18:47:08 2014
 *  Copyright  2008-2014  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __PLUGINS_LASER_SICK_TIM55X_COMMON_AQT_H_
#define __PLUGINS_LASER_SICK_TIM55X_COMMON_AQT_H_

#include "acquisition_thread.h"

#include <config/change_handler.h>

#include <string>
#include <map>

namespace fawkes {
  class Mutex;
}

class SickTiM55xCommonAcquisitionThread
: public LaserAcquisitionThread,
  public fawkes::ConfigurationChangeHandler
{
 public:
  SickTiM55xCommonAcquisitionThread(std::string &cfg_name, std::string &cfg_prefix);
  virtual ~SickTiM55xCommonAcquisitionThread();

  // from LaserAcquisitionThread
  virtual void pre_init(fawkes::Configuration *config, fawkes::Logger *logger);

  void read_common_config();

 protected:
  void init_device();
  void resync();
  void parse_datagram(const unsigned char *datagram, size_t datagram_length);

  virtual void send_with_reply(const char *request, std::string *reply = NULL) = 0;
  virtual void open_device() = 0;
  virtual void close_device() = 0;
  virtual void flush_device() = 0;

private:
  virtual void config_tag_changed(const char *new_tag) { };
  virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v) { };
  virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);
  virtual void config_value_erased(const char *path);

 private:
  bool pre_init_done_;
  float        cfg_time_offset_;

 protected:
  std::string  cfg_name_;
  std::string  cfg_prefix_;

  std::string  dev_model_;

 private:
  unsigned int expected_num_data_;
};


#endif
