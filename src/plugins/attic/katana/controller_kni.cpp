
/***************************************************************************
 *  controller_kni.cpp - KNI Controller class for katana arm
 *
 *  Created: Tue Jan 03 11:40:31 2012
 *  Copyright  2012  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#include "controller_kni.h"

#include "exception.h"

#include <common/MathHelperFunctions.h>
#include <core/exceptions/software.h>

#include <kniBase.h>

namespace fawkes {

/** @class KatanaControllerKni <plugins/katana/controller_kni.h>
 * Controller class for a Neuronics Katana, using libkni to interact
 * with the real Katana arm.
 * @author Bahram Maleki-Fard
 */

/** Constructor. */
KatanaControllerKni::KatanaControllerKni()
: cfg_device_("/dev/ttyS0"), cfg_kni_conffile_("/etc/kni3/hd300/katana6M180.cfg")
{
	cfg_read_timeout_  = 100;
	cfg_write_timeout_ = 0;

	gripper_last_pos_.clear();
	gripper_last_pos_.resize(2);

	x_ = y_ = z_ = 0.;
	phi_ = theta_ = psi_ = 0.;
}

/** Destructor. */
KatanaControllerKni::~KatanaControllerKni()
{
	// Setting to NULL also deletes instance (RefPtr)
	katana_ = NULL;

	device_.reset();
	protocol_.reset();
}

/** Setup parameters needed to initialize Katana arm with libkni.
 * @param device device string, e.g. "/dev/ttyS0"
 * @param kni_conffile path to kni configfile, e.g. "/etc/kni3/hd300/katana6M180.cfg"
 * @param read_timeout timeout for read operations, in ms
 * @param write_timeout timeout for write operations, in ms
 */
void
KatanaControllerKni::setup(std::string &device,
                           std::string &kni_conffile,
                           unsigned int read_timeout,
                           unsigned int write_timeout)
{
	cfg_device_        = device;
	cfg_kni_conffile_  = kni_conffile;
	cfg_read_timeout_  = read_timeout;
	cfg_write_timeout_ = write_timeout;
}

void
KatanaControllerKni::init()
{
	try {
		TCdlCOMDesc ccd = {0, 57600, 8, 'N', 1, (int)cfg_read_timeout_, (int)cfg_write_timeout_};
		device_.reset(new CCdlCOM(ccd, cfg_device_.c_str()));

		protocol_.reset(new CCplSerialCRC());
		protocol_->init(device_.get());

		katana_ = RefPtr<CLMBase>(new CLMBase());
		katana_->create(cfg_kni_conffile_.c_str(), protocol_.get());
		katbase_     = katana_->GetBase();
		sensor_ctrl_ = &katbase_->GetSCT()->arr[0];

		katbase_->recvECH();
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}

	try {
		motor_init_.resize(katana_->getNumberOfMotors());
		for (unsigned int i = 0; i < motor_init_.size(); i++) {
			motor_init_.at(i) = *(katbase_->GetMOT()->arr[i].GetInitialParameters());
		}
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

void
KatanaControllerKni::set_max_velocity(unsigned int vel)
{
	try {
		katana_->setRobotVelocityLimit((int)vel);
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

bool
KatanaControllerKni::final()
{
	if (active_motors_.size() < 1)
		return true;

	bool final = true;
	for (unsigned int i = 0; i < active_motors_.size(); i++) {
		final &= motor_final(active_motors_.at(i));
	}
	cleanup_active_motors();
	return final;
}

bool
KatanaControllerKni::joint_angles()
{
	return true;
}
bool
KatanaControllerKni::joint_encoders()
{
	return true;
}

void
KatanaControllerKni::calibrate()
{
	try {
		katana_->calibrate();
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

void
KatanaControllerKni::stop()
{
	try {
		katana_->freezeRobot();
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

void
KatanaControllerKni::turn_on()
{
	try {
		katana_->switchRobotOn();
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

void
KatanaControllerKni::turn_off()
{
	try {
		katana_->switchRobotOff();
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

void
KatanaControllerKni::read_coordinates(bool refresh)
{
	try {
		katana_->getCoordinates(x_, y_, z_, phi_, theta_, psi_, refresh);
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

void
KatanaControllerKni::read_motor_data()
{
	try {
		if (active_motors_.size() == (unsigned short)katana_->getNumberOfMotors()) {
			katbase_->recvMPS(); // get position for all motors
			katbase_->recvGMS(); // get status flags for all motors
		} else {
			const TKatMOT *mot = katbase_->GetMOT();
			for (unsigned int i = 0; i < active_motors_.size(); i++) {
				mot->arr[active_motors_.at(i)].recvPVP(); // get position data for motor
			}
		}
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

void
KatanaControllerKni::read_sensor_data()
{
	try {
		sensor_ctrl_->recvDAT();
	} catch (/*KNI*/ ::ParameterReadingException &e) {
		throw fawkes::Exception("KNI ParameterReadingException:%s", e.what());
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

void
KatanaControllerKni::gripper_open(bool blocking)
{
	try {
		katana_->openGripper(blocking);
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}

	active_motors_.clear();
	active_motors_.resize(1);
	active_motors_[0] = katbase_->GetMOT()->cnt - 1;

	gripper_last_pos_.clear();
	gripper_last_pos_[0] = katbase_->GetMOT()->arr[active_motors_[0]].GetPVP()->pos;
	gripper_last_pos_[1] = 0; //counter
}

void
KatanaControllerKni::gripper_close(bool blocking)
{
	try {
		katana_->closeGripper(blocking);
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}

	active_motors_.clear();
	active_motors_.resize(1);
	active_motors_[0] = katbase_->GetMOT()->cnt - 1;

	gripper_last_pos_.clear();
	gripper_last_pos_[0] = katbase_->GetMOT()->arr[active_motors_[0]].GetPVP()->pos;
	gripper_last_pos_[1] = 0; //counter
}

void
KatanaControllerKni::move_to(float x,
                             float y,
                             float z,
                             float phi,
                             float theta,
                             float psi,
                             bool  blocking)
{
	cleanup_active_motors();

	try {
		katana_->moveRobotTo(x_, y_, z_, phi_, theta_, psi_, blocking);
	} catch (KNI::NoSolutionException &e) {
		throw fawkes::KatanaNoSolutionException("KNI NoSolutionException:%s", e.what());
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}

	for (short i = 0; i < katana_->getNumberOfMotors(); ++i) {
		add_active_motor(i);
	}
}

void
KatanaControllerKni::move_to(std::vector<int> encoders, bool blocking)
{
	cleanup_active_motors();

	try {
		katana_->moveRobotToEnc(encoders);
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}

	for (unsigned short i = 0; i < encoders.size(); ++i) {
		add_active_motor(i);
	}
}

void
KatanaControllerKni::move_to(std::vector<float> angles, bool blocking)
{
	std::vector<int> encoders;

	try {
		for (unsigned int i = 0; i < angles.size(); i++) {
			encoders.push_back(KNI_MHF::rad2enc((double)angles.at(i),
			                                    motor_init_.at(i).angleOffset,
			                                    motor_init_.at(i).encodersPerCycle,
			                                    motor_init_.at(i).encoderOffset,
			                                    motor_init_.at(i).rotationDirection));
		}
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}

	move_to(encoders, blocking);
}

void
KatanaControllerKni::move_motor_to(unsigned short id, int enc, bool blocking)
{
	if (motor_oor(id))
		throw fawkes::KatanaOutOfRangeException("Motor out of range.");

	cleanup_active_motors();

	try {
		katana_->moveMotorToEnc(id, enc);
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}

	add_active_motor(id);
}

void
KatanaControllerKni::move_motor_to(unsigned short id, float angle, bool blocking)
{
	if (motor_oor(id))
		throw fawkes::KatanaOutOfRangeException("Motor out of range.");

	cleanup_active_motors();

	try {
		katana_->moveMotorTo(id, angle);
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}

	add_active_motor(id);
}

void
KatanaControllerKni::move_motor_by(unsigned short id, int enc, bool blocking)
{
	if (motor_oor(id))
		throw fawkes::KatanaOutOfRangeException("Motor out of range.");

	cleanup_active_motors();

	try {
		katana_->moveMotorByEnc(id, enc);
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}

	add_active_motor(id);
}

void
KatanaControllerKni::move_motor_by(unsigned short id, float angle, bool blocking)
{
	if (motor_oor(id))
		throw fawkes::KatanaOutOfRangeException("Motor out of range.");

	cleanup_active_motors();

	try {
		katana_->moveMotorBy(id, angle);
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}

	add_active_motor(id);
}

// getters
double
KatanaControllerKni::x()
{
	return x_;
}

double
KatanaControllerKni::y()
{
	return y_;
}

double
KatanaControllerKni::z()
{
	return z_;
}

double
KatanaControllerKni::phi()
{
	return phi_;
}

double
KatanaControllerKni::theta()
{
	return theta_;
}

double
KatanaControllerKni::psi()
{
	return psi_;
}

void
KatanaControllerKni::get_sensors(std::vector<int> &to, bool refresh)
{
	if (refresh)
		read_sensor_data();

	try {
		const TSctDAT *sensor_data = sensor_ctrl_->GetDAT();

		const int num_sensors = (size_t)sensor_data->cnt;
		to.clear();
		to.resize(num_sensors);

		for (int i = 0; i < num_sensors; ++i) {
			to[i] = sensor_data->arr[i];
		}
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

void
KatanaControllerKni::get_encoders(std::vector<int> &to, bool refresh)
{
	try {
		std::vector<int> encoders = katana_->getRobotEncoders(refresh);

		to.clear();
		to = encoders;
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

void
KatanaControllerKni::get_angles(std::vector<float> &to, bool refresh)
{
	try {
		std::vector<int> encoders = katana_->getRobotEncoders(refresh);

		to.clear();
		for (unsigned int i = 0; i < encoders.size(); i++) {
			to.push_back(KNI_MHF::enc2rad(encoders.at(i),
			                              motor_init_.at(i).angleOffset,
			                              motor_init_.at(i).encodersPerCycle,
			                              motor_init_.at(i).encoderOffset,
			                              motor_init_.at(i).rotationDirection));
		}
	} catch (/*KNI*/ ::Exception &e) {
		throw fawkes::Exception("KNI Exception:%s", e.what());
	}
}

bool
KatanaControllerKni::motor_oor(unsigned short id)
{
	return id > (unsigned short)katana_->getNumberOfMotors();
}

bool
KatanaControllerKni::motor_final(unsigned short id)
{
	CMotBase mot = katbase_->GetMOT()->arr[id];
	if (mot.GetPVP()->msf == MSF_MOTCRASHED)
		throw fawkes::KatanaMotorCrashedException("Motor %u crashed.", id);

	// extra check for gripper, consider final if not moved for a while
	unsigned short gripper_not_moved = 0;
	if (id == katbase_->GetMOT()->cnt - 1) {
		if (gripper_last_pos_[0] == mot.GetPVP()->pos) {
			gripper_last_pos_[1] += 1;
		} else {
			gripper_last_pos_[0] = mot.GetPVP()->pos;
			gripper_last_pos_[1] = 0;
		}
		gripper_not_moved = gripper_last_pos_[1];
	}

	return (std::abs(mot.GetTPS()->tarpos - mot.GetPVP()->pos) < 10) or (gripper_not_moved > 3);
}

void
KatanaControllerKni::cleanup_active_motors()
{
	for (unsigned int i = 0; i < active_motors_.size(); ++i) {
		if (motor_final(active_motors_.at(i))) {
			active_motors_.erase(active_motors_.begin() + i);
			--i;
		}
	}
}

void
KatanaControllerKni::add_active_motor(unsigned short id)
{
	for (unsigned int i = 0; i < active_motors_.size(); ++i) {
		if (active_motors_.at(i) == id) {
			return;
		}
	}
	active_motors_.push_back(id);
}

} // end of namespace fawkes
