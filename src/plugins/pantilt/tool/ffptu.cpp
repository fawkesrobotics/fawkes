
/***************************************************************************
 *  ffptu.cpp - Control PTU via keyboard
 *
 *  Created: Thu Oct 06 16:28:16 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "../robotis/rx28.h"

#include <blackboard/remote.h>
#include <core/exceptions/system.h>
#include <interface/interface_info.h>
#include <interfaces/PanTiltInterface.h>
#include <logging/console.h>
#include <utils/system/argparser.h>
#include <utils/system/getkey.h>
#include <utils/time/time.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

using namespace fawkes;

void
print_usage(const char *program_name)
{
	printf("Usage: %s [-h] [-r host[:port]] <command>\n"
	       " Options:\n"
	       " -h              This help message\n"
	       " -r host[:port]  Remote host (and optionally port) to connect to\n"
	       " -p <ptu>        PTU, interface must have id 'PanTilt <ptu>'\n"
	       " -l              List available PTUs (only on remote blackboard)\n"
	       " -i              Invert tilt control buttons\n\n"
	       " Commands:\n"
	       "move [pan P] [tilt T]\n"
	       "  Move PTU, if either pan or tilt are supplied, send the\n"
	       "  command and wait for angle to be reached. Otherwise enter\n"
	       "  interactive mode\n"
	       "rx28-set-id <old-id> <new-id>\n"
	       "  Send message to set a new servo ID.\n"
	       "  WARNING: use this with only a single servo connected!\n"
	       "rx28-discover\n"
	       "  Discover and print servos on the BUS.\n",
	       program_name);
}

/** Remote control PTUs via keyboard.
 * @author Tim Niemueller
 */
class PTUJoystickControl
{
public:
	/** Constructor.
   * @param argc number of arguments in argv
   * @param argv array of parameters passed into the program
   */
	PTUJoystickControl(int argc, char **argv) : argp_(argc, argv, "hr:p:li")
	{
		rx28_       = NULL;
		bb_         = NULL;
		ptu_if_     = NULL;
		resolution_ = 0.1f;

		if (argp_.has_arg("h")) {
			print_usage(argv[0]);
			exit(0);
		}
	}

	/** Destructor. */
	~PTUJoystickControl()
	{
		if (bb_) {
			bb_->close(ptu_if_);
			delete bb_;
		}
		delete rx28_;
	}

	/** Initialize BB connection. */
	void
	init_bb()
	{
		char *             host      = (char *)"localhost";
		unsigned short int port      = 1910;
		bool               free_host = argp_.parse_hostport("r", &host, &port);

		bb_ = new RemoteBlackBoard(host, port);
		if (free_host)
			free(host);

		if (argp_.has_arg("l")) {
			InterfaceInfoList *l = bb_->list("PanTiltInterface", "PanTilt *");
			if (l->empty()) {
				printf("No interfaces found");
			}
			for (InterfaceInfoList::iterator i = l->begin(); i != l->end(); ++i) {
				std::string id = i->id();
				id             = id.substr(std::string("PanTilt ").length());

				printf("PTU: %s   Writer: %s\n", id.c_str(), i->has_writer() ? "Yes" : "No");
			}
			delete l;
			return;
		}

		if (argp_.has_arg("p")) {
			std::string iface_id = std::string("PanTilt ") + argp_.arg("p");
			ptu_if_              = bb_->open_for_reading<PanTiltInterface>(iface_id.c_str());
		} else {
			InterfaceInfoList *l = bb_->list("PanTiltInterface", "PanTilt *");
			if (l->empty()) {
				throw Exception("No PanTilt interface opened on remote!");
			}
			for (InterfaceInfoList::iterator i = l->begin(); i != l->end(); ++i) {
				if (i->has_writer()) {
					ptu_if_ = bb_->open_for_reading<PanTiltInterface>(i->id());
				} else {
					printf("Interface %s has no writer, ignoring\n", i->id());
				}
			}
			delete l;
		}

		if (!ptu_if_) {
			throw Exception("No suitable PanTiltInterface found");
		}
	}

	/** Initialize Robotis RX28 raw servo access. */
	void
	init_rx28()
	{
		rx28_                        = new RobotisRX28("/dev/ttyUSB0");
		RobotisRX28::DeviceList devl = rx28_->discover();

		if (devl.empty()) {
			throw Exception("No devices found\n");
		}
	}

	/** Run control loop. */
	void
	run()
	{
		if (argp_.has_arg("l")) {
			init_bb();
		} else if (argp_.num_items() == 0) {
			interactive_move();
		} else if (strcmp(argp_.items()[0], "move") == 0) {
			if (argp_.num_items() == 1) {
				interactive_move();
			} else {
				exec_move();
			}
		} else if (strcmp(argp_.items()[0], "rx28-set-id") == 0) {
			rx28_set_id();

		} else if (strcmp(argp_.items()[0], "rx28-discover") == 0) {
			rx28_discover();

		} else {
			printf("Unknown command '%s'\n", argp_.items()[0]);
		}
	}

private:
	void
	interactive_move()
	{
		init_bb();

		Time last, now;

		char tilt_up   = 65;
		char tilt_down = 66;
		if (argp_.has_arg("i"))
			std::swap(tilt_up, tilt_down);

		float pan, tilt, new_pan, new_tilt;
		float speed = 0.0, new_speed = 0.5;

		ptu_if_->read();
		pan = new_pan = ptu_if_->pan();
		tilt = new_tilt = ptu_if_->tilt();

		last.stamp();
		char key       = 0;
		int  wait_time = 5;
		while (key != 'q') {
			key = getkey(wait_time);
			//printf("Key: %u = %u\n", key, key);
			if (key != 0) {
				now.stamp();
				if ((now - &last) < 0.5) {
					wait_time = 1;
				}
				last.stamp();
			}
			if (key == 0) {
				wait_time = 5;

			} else if (key == 27) {
				key = getkey();
				if (key == 0) {
					// Escape key
					new_pan  = pan;
					new_tilt = tilt;
				} else {
					if (key != 91)
						continue;

					key = getkey();
					if (key == 0)
						continue;

					if (key == tilt_up) {
						new_tilt = std::min(tilt + resolution_, ptu_if_->max_tilt());
					} else if (key == tilt_down) {
						new_tilt = std::max(tilt - resolution_, ptu_if_->min_tilt());
					} else if (key == 67) {
						new_pan = std::max(pan - resolution_, ptu_if_->min_pan());
					} else if (key == 68) {
						new_pan = std::min(pan + resolution_, ptu_if_->max_pan());
					} else
						continue;
				}
			} else if (key == '0') {
				new_pan = new_tilt = 0.f;
			} else if (key == '9') {
				new_pan  = 0;
				new_tilt = M_PI / 2.;
			} else if (key == 'r') {
				resolution_ = 0.1f;
			} else if (key == 'R') {
				resolution_ = 0.01f;
			} else if (key == '+') {
				new_speed = std::min(speed + 0.1, 1.0);
			} else if (key == '-') {
				new_speed = std::max(speed - 0.1, 0.0);
			}

			if (speed != new_speed) {
				speed          = new_speed;
				float pan_vel  = speed * ptu_if_->max_pan_velocity();
				float tilt_vel = speed * ptu_if_->max_tilt_velocity();

				printf("Setting velocity %f/%f (max %f/%f)\n",
				       pan_vel,
				       tilt_vel,
				       ptu_if_->max_pan_velocity(),
				       ptu_if_->max_tilt_velocity());

				PanTiltInterface::SetVelocityMessage *svm =
				  new PanTiltInterface::SetVelocityMessage(pan_vel, tilt_vel);
				ptu_if_->msgq_enqueue(svm);
			}

			if ((pan != new_pan) || (tilt != new_tilt)) {
				pan  = new_pan;
				tilt = new_tilt;

				printf("Goto %f/%f\n", pan, tilt);

				PanTiltInterface::GotoMessage *gm = new PanTiltInterface::GotoMessage(pan, tilt);
				ptu_if_->msgq_enqueue(gm);
			}
		}
	}

	void
	exec_move()
	{
		init_bb();

		float pan, tilt, new_pan, new_tilt;

		ptu_if_->read();
		pan = new_pan = ptu_if_->pan();
		tilt = new_tilt = ptu_if_->tilt();

		const std::vector<const char *> &items = argp_.items();
		for (unsigned int i = 1; i < items.size(); ++i) {
			if (strcmp(items[i], "pan") == 0) {
				if (items.size() > i + 1) {
					new_pan = argp_.parse_item_float(++i);
				} else {
					printf("No pan value supplied, aborting.\n");
					return;
				}
			} else if (strcmp(items[i], "tilt") == 0) {
				if (items.size() > i + 1) {
					new_tilt = argp_.parse_item_float(++i);
				} else {
					printf("No tilt value supplied, aborting.\n");
					return;
				}
			} else {
				printf("Unknown parameter '%s', aborting.\n", items[i]);
				return;
			}
		}

		if ((pan != new_pan) || (tilt != new_tilt)) {
			printf("Goto pan %f and tilt %f\n", new_pan, new_tilt);

			PanTiltInterface::SetVelocityMessage *svm =
			  new PanTiltInterface::SetVelocityMessage(ptu_if_->max_pan_velocity() / 2.,
			                                           ptu_if_->max_tilt_velocity() / 2.);
			ptu_if_->msgq_enqueue(svm);

			PanTiltInterface::GotoMessage *gm = new PanTiltInterface::GotoMessage(new_pan, new_tilt);
			ptu_if_->msgq_enqueue(gm);
			usleep(5e5);
		}
	}

	void
	rx28_set_id()
	{
		init_rx28();

		int old_id = argp_.parse_item_int(1);
		int new_id = argp_.parse_item_int(2);

		printf("Servo IDs *before* setting the ID:\n");
		RobotisRX28::DeviceList devl = rx28_->discover();
		for (RobotisRX28::DeviceList::iterator i = devl.begin(); i != devl.end(); ++i) {
			printf("  %d\n", *i);
		}

		if (old_id < 0 || old_id >= RobotisRX28::BROADCAST_ID) {
			printf("Invalid old ID %i, must be in range [0..%u]\n", old_id, RobotisRX28::BROADCAST_ID);
			return;
		}

		if (new_id < 0 || new_id >= RobotisRX28::BROADCAST_ID) {
			printf("Invalid new ID %i, must be in range [0..%u]\n", new_id, RobotisRX28::BROADCAST_ID);
			return;
		}

		rx28_->set_id(old_id, new_id);

		printf("Servo IDs *after* setting the ID:\n");
		devl = rx28_->discover();
		for (RobotisRX28::DeviceList::iterator i = devl.begin(); i != devl.end(); ++i) {
			printf("  %d\n", *i);
		}
	}

	void
	rx28_discover()
	{
		init_rx28();

		printf("Servo IDs on the bus:\n");
		RobotisRX28::DeviceList devl = rx28_->discover();
		for (RobotisRX28::DeviceList::iterator i = devl.begin(); i != devl.end(); ++i) {
			printf("  %d\n", *i);
		}
	}

private:
	ArgumentParser    argp_;
	BlackBoard *      bb_;
	PanTiltInterface *ptu_if_;
	float             resolution_;
	RobotisRX28 *     rx28_;
};

/** Config tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
	try {
		PTUJoystickControl ptuctrl(argc, argv);
		ptuctrl.run();
	} catch (Exception &e) {
		printf("Running failed: %s\n\n", e.what());
		print_usage(argv[0]);
		exit(0);
	}

	return 0;
}
