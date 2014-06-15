
/***************************************************************************
 *  sick_tim55x_aqt.h - Thread to retrieve laser data from Sick TiM 55x
 *
 *  Created: Tue Jun 10 16:47:50 2014
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

#ifndef __PLUGINS_LASER_SICK_TIM55X_USB_AQT_H_
#define __PLUGINS_LASER_SICK_TIM55X_USB_AQT_H_

#include "acquisition_thread.h"

#include <string>
#include <map>
#include <libusb.h>

namespace fawkes {
  class TimeWait;
  class Mutex;
}

class SickTiM55xUSBAcquisitionThread : public LaserAcquisitionThread
{
 public:
  SickTiM55xUSBAcquisitionThread(std::string &cfg_name, std::string &cfg_prefix);

  // from LaserAcquisitionThread
  virtual void pre_init(fawkes::Configuration *config, fawkes::Logger *logger);

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  void init_device();
  libusb_device_handle *  open_device(uint16_t vendor_id, uint16_t product_id,
				      std::string &serial);
  void send_with_reply(const char *request, std::string *reply = NULL);
  void parse_datagram(const unsigned char *datagram, size_t datagram_length);

 private:
  bool __pre_init_done;

  std::string  __cfg_name;
  std::string  __cfg_prefix;
  std::string  __cfg_serial;
  float        __cfg_time_offset;

  libusb_context *usb_ctx_;
  libusb_device_handle  *usb_device_handle_;
  fawkes::Mutex  *usb_mutex_;
};


#endif
