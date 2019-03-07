
/***************************************************************************
 *  urg_aqt.h - Thread to retrieve laser data from Hokuyo URG
 *
 *  Created: Sat Nov 28 01:29:48 2009
 *  Copyright  2008-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_LASER_URG_AQT_H_
#define _PLUGINS_LASER_URG_AQT_H_

#include "acquisition_thread.h"

#include <map>
#include <string>

namespace qrk {
class UrgCtrl;
}

namespace fawkes {
class TimeWait;
}

class HokuyoUrgAcquisitionThread : public LaserAcquisitionThread
{
public:
	HokuyoUrgAcquisitionThread(std::string &cfg_name, std::string &cfg_prefix);

	// from LaserAcquisitionThread
	virtual void pre_init(fawkes::Configuration *config, fawkes::Logger *logger);

	virtual void init();
	virtual void finalize();
	virtual void loop();

private:
	std::map<std::string, std::string> get_device_info(qrk::UrgCtrl *ctrl);

private:
	bool          pre_init_done_;
	unsigned int  number_of_values_;
	qrk::UrgCtrl *ctrl_;
	int           fd_;

	fawkes::TimeWait *timer_;

	std::string cfg_name_;
	std::string cfg_prefix_;

	std::map<std::string, std::string> device_info_;

	std::string cfg_device_;
	std::string cfg_serial_;
	float       cfg_time_offset_;

	unsigned int first_ray_;
	unsigned int last_ray_;
	unsigned int front_ray_;
	unsigned int slit_division_;
	float        step_per_angle_;
	float        angle_per_step_;
	float        angular_range_;
	long int     scan_msec_;
};

#endif
