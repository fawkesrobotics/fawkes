
/***************************************************************************
 *  deadspots.cpp - Laser dead spots calibration tool
 *
 *  Created: Wed Jun 24 12:00:54 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/thread.h>
#include <core/threading/wait_condition.h>
#include <utils/system/argparser.h>
#include <utils/time/time.h>
#include <netcomm/fawkes/client.h>
#include <blackboard/remote.h>
#include <blackboard/interface_listener.h>
#include <config/netconf.h>

#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <utility>

#define MAX_WAIT_TIME 60
#define DEFAULT_WAIT_TIME 10
#define DEFAULT_NUM_MEASUREMENTS 100
#define DEFAULT_COMPARE_DISTANCE 0.9

#define INITIAL_MEASUREMENT 123456.0

using namespace fawkes;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-h] [-r host[:port]] <num_spots> <config_prefix>\n"
	 " -h              This help message\n"
	 " -r host[:port]  Remote host (and optionally port) to connect to\n"
	 " -n <NUM>        Number of measurements to use, defaults to %u\n"
	 " -w <SEC>        Wait time in seconds, defaults to %u\n"
	 " -c <DIST>       Compare distance in m, defaults to %f\n"
	 " -m <MARGIN_DEG> Margin in degree to add around dead spot regions\n"
	 " -d              Dry-run, do not save results to configuration\n"
	 " -b              Show data by opening a blackboard laser interface\n"
	 " -i <ID>         Open laser interface named <ID>\n"
	 "<num_spots>      Expected number of dead spots\n",
	 program_name, DEFAULT_NUM_MEASUREMENTS, DEFAULT_WAIT_TIME,
	 DEFAULT_COMPARE_DISTANCE);
}

/** Calibrator for dead ranges.
 * Depending how the laser is mounted parts of the range it covers might be
 * useless data, for example if hidden behind rods. This calibrator detects
 * those ranges and writes the information to the config suitable to be
 * used by the LaserDeadSpotsDataFilter.
 * @author Tim Niemueller
 */
class LaserDeadSpotCalibrator : public BlackBoardInterfaceListener
{
 public:
  /** Constructor.
   * @param num_spots number of expected spots
   * @param num_measurements number of measurements to take
   * @param compare_distance distance to compare values to
   * @param margin extra margin in degree to add around detected regions
   * @param blackboard blackboard to register with as listener
   * @param laser360 360 beams laser interface
   * @param laser720 720 beams laser interface
   */
  LaserDeadSpotCalibrator(unsigned int num_spots, unsigned int num_measurements,
			  float compare_distance, float margin,
			  BlackBoard *blackboard,
			  Laser360Interface *laser360, Laser720Interface *laser720)
    : BlackBoardInterfaceListener("LaserDeadSpotCalibrator")
  {
    laser720_           = laser720;
    laser360_           = laser360;
    blackboard_         = blackboard;
    num_spots_expected_ = num_spots;
    num_measurements_   = num_measurements;
    cur_measurement_    = 0;
    num_beams_          = 0;
    margin_             = margin;
    compare_distance_   = compare_distance;
    measurements_.clear();
    num_spots_found_    = 0;

    if (!laser720_ || ! laser720_->has_writer()) {
      lowres_calibrate_ = true;
      num_beams_ = laser360_->maxlenof_distances();
      bbil_add_data_interface(laser360_);
    } else {
      lowres_calibrate_ = false;
      num_beams_ = laser720_->maxlenof_distances();
      bbil_add_data_interface(laser720_);
    }
    std::vector<float> tmp;
    tmp.resize(num_measurements_, INITIAL_MEASUREMENT);
    measurements_.resize(num_beams_, tmp);
  
    blackboard_->register_listener(this);
  }

  /** Wait for the calibration to be finished. */
  void
  wait_finished()
  {
    start_measuring_ = true;
    finish_waitcond_.wait();
  }

  /** Get spots.
   * @return vector of detected dead regions
   */
  std::vector<std::pair<float, float> >
  get_dead_spots()
  {
    return dead_spots_;
  }

  /** Get number of spots.
   * @return number of spots
   */
  unsigned int
  num_detected_spots()
  {
    return num_spots_found_;
  }

 private:
  float
  calculate_median(std::vector<float> measurements)
  {
    std::sort(measurements.begin(), measurements.end());
    return measurements[measurements.size() / 2];
  }

  std::vector<float>
  calculate_medians()
  {
    std::vector<float> rv;
    rv.resize(num_beams_, INITIAL_MEASUREMENT);

    for (unsigned int i = 0; i < measurements_.size(); ++i) {
      rv[i] = calculate_median(measurements_[i]);
    }

    return rv;
  }


  void
  analyze()
  {
    //printf("ANALYZING\n");
    float angle_factor = 360.0 / num_beams_;

    std::vector<float> medians = calculate_medians();

    bool iteration_done = false;
    for (unsigned int i = 0; ! iteration_done && i < medians.size(); ++i) {

      if (medians[i] == INITIAL_MEASUREMENT) {
	printf("WARNING: No valid measurement at angle %f°!\n", i * angle_factor);
	continue;
      }

      if (medians[i] < compare_distance_) {
	// start of spot, look for end
	float start_angle = i * angle_factor;

	//printf("Region starting at %f\n", start_angle);

	do {
	  //printf("Median %u: %f < %f\n", i, medians[i], compare_distance_);

	  if ((i + 1) >= medians.size()) {
	    if (iteration_done) {
	      printf("Could not find end for region starting at %f°, all values "
		     "too short?\n", start_angle);
	      break;
	    } else {
	      iteration_done = true;
	      i = 0;
	    }
	  } else {
	    ++i;
	  }
	} while ((medians[i] < compare_distance_) && (medians[i] != INITIAL_MEASUREMENT));
	if (medians[i] >= compare_distance_) {
	  float end_angle = i * angle_factor;
	  //printf("Region ends at %f\n", end_angle);
	  dead_spots_.push_back(std::make_pair(start_angle, end_angle));
	} else {
	  // did not find end of region
	  break;
	}
      }
    }
  }

  void
  sort_spots()
  {
    std::sort(dead_spots_.begin(), dead_spots_.end());
  }

  bool
  merge_region(unsigned int ind1, unsigned int ind2)
  {
    if (dead_spots_[ind1].second >= dead_spots_[ind2].first) {
      // regions overlap, merge!
      if (dead_spots_[ind1].first > dead_spots_[ind2].second) {
	// merging would create a region across the discontinuity, do a
	// split-merge, i.e. join regions to one, but save as two (cf. normalize())
	//printf("Merging overlapping regions %u [%f, %f] and %u [%f, %f] to [%f, %f]/[%f, %f]\n",
	//       ind1, dead_spots_[ind1].first, dead_spots_[ind1].second,
	//       ind2, dead_spots_[ind2].first, dead_spots_[ind2].second,
	//       dead_spots_[ind1].first, 360., 0., dead_spots_[ind2].second);
	dead_spots_[ind1].second  = 360.;
	dead_spots_[ind2].first = 0.;
      } else {
	//printf("Merging overlapping regions %u [%f, %f] and %u [%f, %f] to [%f, %f]\n",
	//       ind1, dead_spots_[ind1].first, dead_spots_[ind1].second,
	//       ind2, dead_spots_[ind2].first, dead_spots_[ind2].second,
	//       dead_spots_[ind1].first, dead_spots_[ind2].second);
	dead_spots_[ind1].second = dead_spots_[ind2].second;
	dead_spots_.erase(dead_spots_.begin() + ind2);
	return false;
      }
    }
    return true;
  }

  void
  merge_spots()
  {
    //printf("MERGING\n");
    unsigned int i = 0;
    while (i < dead_spots_.size() - 1) {
      //printf("Comparing %u, %u, %f >= %f, %zu\n", i, i+1,
      //       dead_spots_[i].second, dead_spots_[i+1].first, dead_spots_.size());
      if (merge_region(i, i+1))  ++i;
    }
    // now check for posssible merge of first and last region (at the discontinuity
    unsigned int last = dead_spots_.size() - 1;
    if ((dead_spots_[last].second >= dead_spots_[0].first) && (dead_spots_[last].second <= dead_spots_[0].second) &&
	(dead_spots_[0].first >= dead_spots_[last].first - 360) && (dead_spots_[0].second <= dead_spots_[last].second)) {
      merge_region(last, 0);
    }
  }

  void
  apply_margin()
  {
    //printf("MARGIN\n");
    if (margin_ != 0.0) {
      // post-process, add margins, possibly causing regions to be merged
      // add margins
      for (unsigned int i = 0; i != dead_spots_.size(); ++i) {
	//float before_start = dead_spots_[i].first;
	//float before_end   = dead_spots_[i].second;
	dead_spots_[i].first  -= margin_;
	dead_spots_[i].second += margin_;
	if (dead_spots_[i].second > 360.0) {
	  dead_spots_[i].second -= 360.0;
	}
	//printf("Applying margin to spot %i, [%f, %f] -> [%f, %f]\n",
	//       i, before_start, before_end,
	//       dead_spots_[i].first, dead_spots_[i].second);
      }
      // look if regions need to be merged
      merge_spots();
    }
  }

  void
  normalize()
  {
    //printf("NORMALIZING\n");
    // normalize
    for (unsigned int i = 0; i != dead_spots_.size(); ++i) {
      if (dead_spots_[i].first < 0.) {
	//printf("Normalizing %i start angle %f -> %f\n", i,
	//       dead_spots_[i].first, 360. + dead_spots_[i].first);
	dead_spots_[i].first = 360. + dead_spots_[i].first;
      }
      if (dead_spots_[i].second < 0.) {
	//printf("Normalizing %i end angle %f -> %f\n", i,
	//       dead_spots_[i].second, 360. + dead_spots_[i].second);
	dead_spots_[i].second = 360. + dead_spots_[i].first;
      }

      if (dead_spots_[i].first > dead_spots_[i].second) {
	// range over the discontinuity at 0°/360°, split into two regions
	//printf("Splitting (size %zu) region %i from [%f, %f] ", dead_spots_.size(), i,
	//       dead_spots_[i].first, dead_spots_[i].second);
	dead_spots_.resize(dead_spots_.size() + 1);
	for (int j = dead_spots_.size()-1; j >= (int)i; --j) {
	  dead_spots_[j+1] = dead_spots_[j];
	}
	dead_spots_[i+1].first = 0;
	dead_spots_[i].second  = 360.0;

	//printf("to [%f, %f] and [%f, %f] (size %zu)\n", dead_spots_[i].first, dead_spots_[i].second,
	//       dead_spots_[i+1].first, dead_spots_[i+1].second, dead_spots_.size());
      }
    }
    //print_spots();
    sort_spots();
    merge_spots();
  }


  void
  print_spots()
  {
    for (unsigned int i = 0; i != dead_spots_.size(); ++i) {
      printf("Spot %u   start: %3.2f   end: %3.2f\n", i,
	     dead_spots_[i].first, dead_spots_[i].second);
    }
  }

  void
  process_measurements()
  {
    analyze();

    if (dead_spots_.size() > 0) {
      apply_margin();
      print_spots();

      num_spots_found_ = dead_spots_.size();
      normalize();
    } else {
      num_spots_found_ = 0;
    }

    if (num_spots_found_ != num_spots_expected_) {
      printf("Error, expected %u dead spots, but detected %u.\n",
	     num_spots_expected_, num_spots_found_);
      print_spots();
    } else {
      printf("Found expected number of %u dead spots\n", num_spots_expected_);
      if (dead_spots_.size() > num_spots_expected_) {
	printf("Note that more regions will be printed than spots were expected.\n"
	       "This is due to splitting that occurs around the discontinuity at 0°/360°\n");
      }
      if (num_spots_expected_ > dead_spots_.size()) {
	printf("Note that less regions will be printed than spots were expected.\n"
	       "This is due to merging that occurred after applying the margin you\n"
	       "suggested and normalizing the data.\n");
      }
      print_spots();
    }
  }

  virtual void
  bb_interface_data_changed(Interface *interface) throw()
  {
    if (! start_measuring_)  return;

    printf("\r%3u samples remaining...", num_measurements_ - cur_measurement_);
    fflush(stdout);

    float *distances = NULL;
    unsigned int num_distances = 0;
    if (lowres_calibrate_) {
      laser360_->read();
      distances     = laser360_->distances();
      num_distances = laser360_->maxlenof_distances();
    } else {
      laser720_->read();
      distances     = laser720_->distances();
      num_distances = laser720_->maxlenof_distances();
    }

    for (unsigned int i = 0; i < num_distances; ++i) {
      if (finite(distances[i]) && distances[i] > 1e-6) {
	measurements_[i][cur_measurement_] = distances[i];
      }
    }

    if (++cur_measurement_ >= num_measurements_) {
      printf("\rMeasuring done, post-processing data now.\n");
      process_measurements();
      blackboard_->unregister_listener(this);
      finish_waitcond_.wake_all();
    }
  }

 private:
  BlackBoard        *blackboard_;
  Laser360Interface *laser360_;
  Laser720Interface *laser720_;
  WaitCondition      finish_waitcond_;

  float              margin_;
  bool               lowres_calibrate_;
  bool               start_measuring_;
  unsigned int       num_spots_expected_;
  unsigned int       num_beams_;
  unsigned int       num_measurements_;
  unsigned int       cur_measurement_;
  unsigned int       num_spots_found_;
  float              compare_distance_;
  std::vector<std::vector<float> >      measurements_;
  std::vector<std::pair<float, float> > dead_spots_;
};

int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "hr:n:w:c:m:bdi:");

  if ( argp.has_arg("h") ) {
    print_usage(argv[0]);
    exit(0);
  }

  char *host                = (char *)"localhost";
  unsigned short int port   = FAWKES_TCP_PORT;
  long int num_measurements = DEFAULT_NUM_MEASUREMENTS;
  long int wait_time        = DEFAULT_WAIT_TIME;
  float compare_distance    = DEFAULT_COMPARE_DISTANCE;
  float margin              = 0;
  std::string interface_id  = "Laser";
  std::string cfg_prefix    = "";

  if (argp.has_arg("n")) {
    num_measurements = argp.parse_int("n");
    if (num_measurements <= 0) {
      printf("Invalid number of measurements, must be > 0\n\n");
      print_usage(argp.program_name());
      return -4;
    }
  }
  if (argp.has_arg("w")) {
    wait_time = argp.parse_int("w");
    if (wait_time < 0) {
      printf("Invalid wait time, must be integer > 0\n\n");
      print_usage(argp.program_name());
      return -4;
    } else if (wait_time > MAX_WAIT_TIME) {
      printf("Wait time of more than %u seconds are nonsense, aborting.\n\n", MAX_WAIT_TIME);
      print_usage(argp.program_name());
      return -4;
    }
  }
  if (argp.has_arg("c")) {
    compare_distance = argp.parse_float("c");
    if (compare_distance < 0) {
      printf("Invalid compare distance, must be > 0\n\n");
      print_usage(argp.program_name());
      return -4;
    }
  }
  if (argp.has_arg("m")) {
    margin = argp.parse_int("m");
    if ((margin <= -360) || (margin >= 360)) {
      printf("Invalid margin, must be in the ragen [-359, 359]\n\n");
      print_usage(argp.program_name());
      return -4;
    }
  }
  if (argp.num_items() == 0) {
    printf("Number of expected dead spots not supplied\n\n");
    print_usage(argp.program_name());
    return -4;
  } else if ((argp.num_items() == 1) && ! argp.has_arg("d") ) {
    printf("Config prefix not given and not dry-run\n\n");
    print_usage(argp.program_name());
    return -4;
  } else if (argp.num_items() > 2) {
    printf("Too many arguments\n\n");
    print_usage(argp.program_name());
    return -4;
  } else if (argp.num_items() == 2) {
    cfg_prefix = argp.items()[1];
    if (cfg_prefix[cfg_prefix.length() - 1] != '/') {
      cfg_prefix += "/";
    }
  }

  if (argp.has_arg("i")) {
    interface_id =  argp.arg("i");
  }
  bool free_host = argp.parse_hostport("r", &host, &port);

  FawkesNetworkClient *client;
  BlackBoard *blackboard;
  NetworkConfiguration *netconf;

  try {
    client     = new FawkesNetworkClient(host, port);
    client->connect();
    blackboard = new RemoteBlackBoard(client);
    netconf    = new NetworkConfiguration(client);
  } catch (Exception &e) {
    printf("Failed to connect to remote host at %s:%u\n\n", host, port);
    e.print_trace();
    return -1;
  }

  if ( free_host )  free(host);

  Laser360Interface *laser360 = NULL;
  Laser720Interface *laser720 = NULL;
  try {
    laser360 = blackboard->open_for_reading<Laser360Interface>(interface_id.c_str());
    laser720 = blackboard->open_for_reading<Laser720Interface>(interface_id.c_str());
  } catch (Exception &e) {
    printf("Failed to open blackboard interfaces");
    e.print_trace();
    //return -2;
  }

  if (! laser720->has_writer() && ! laser360->has_writer() ) {
    printf("No writer for neither high nor low resolution laser.\n"
	   "Laser plugin not loaded?\n\n");
    blackboard->close(laser360);
    blackboard->close(laser720);
    //return -3;
  }

  if (! laser720->has_writer()) {
    printf("Warning: high resolution laser not found calibrating with 1° resolution.\n"
	   "         It is recommended to enable the high resolution mode for\n"
	   "         calibration. Acquired 1° data may be unsuitable when used in\n"
	   "         high resolution mode!\n\n");
    blackboard->close(laser720);
    laser720 = NULL;
  }

  Time now, start;
  start.stamp();
  now.stamp();

  printf("Position the laser such that it has %f m of free space around it.\n\n"
	 "Also verify that the laser is running with disable filters\n\n", compare_distance);
  fflush(stdout);
  float diff = 0;
  while ((diff = (now - &start)) < wait_time) {
    printf("\rCalibration will start in %2li sec (Ctrl-C to abort)...", wait_time - (unsigned int)floor(diff));
    fflush(stdout);
    usleep(200000);
    now.stamp();
  }
  printf("\rCalibration starting now.                                    \n");

  unsigned int num_spots = argp.parse_item_int(0);

  LaserDeadSpotCalibrator *calib;
  calib = new LaserDeadSpotCalibrator(num_spots, num_measurements, compare_distance, margin,
				      blackboard, laser360, laser720);
  calib->wait_finished();

  std::vector<std::pair<float, float> > dead_spots = calib->get_dead_spots();

  if ( ! argp.has_arg("d") ) {
    if ( num_spots != calib->num_detected_spots() ) {
      printf("Number of spots does not match expectation. Not writing to config file.");
    } else {
      printf("Storing information in remote config\n");

      netconf->set_mirror_mode(true);

      for (unsigned int i = 0; i < 2; ++i) {
	// do twice, after erasing host specific values there might be default
	// values
	Configuration::ValueIterator *vit = netconf->search("/hardware/laser/dead_spots/");
	while (vit->next()) {
	  //printf("Erasing existing value %s\n", vit->path());
	  if (vit->is_default()) {
	    netconf->erase_default(vit->path());
	  } else {
	    netconf->erase(vit->path());
	  }
	}
	delete vit;
      }

      for (unsigned int i = 0; i < dead_spots.size(); ++i) {
	char *prefix;
	if (asprintf(&prefix, "%s%u/", cfg_prefix.c_str(), i) == -1) {
	  printf("Failed to store dead spot %u, out of memory\n", i);
	  continue;
	}
	std::string start_path = std::string(prefix) + "start";
	std::string end_path   = std::string(prefix) + "end";
	free(prefix);
	netconf->set_float(start_path.c_str(), dead_spots[i].first);
	netconf->set_float(end_path.c_str(), dead_spots[i].second);
      }
    }
  }

  delete calib;
  delete netconf;
  blackboard->close(laser360);
  blackboard->close(laser720);

  if (argp.has_arg("b")) {
    Laser720Interface *lcalib = blackboard->open_for_writing<Laser720Interface>("Laser Calibration");
    for (unsigned int i = 0; i < 720; ++i) {
      lcalib->set_distances(i, 1.0);
    }
    for (unsigned int i = 0; i != dead_spots.size(); ++i) {
      const unsigned int start = (unsigned int)dead_spots[i].first * 2;
      unsigned int end   = (unsigned int)dead_spots[i].second * 2;
      if (end == 720) end = 719;
      //printf("Marking dead %f/%u to %f/%u\n",
      //     dead_spots[i].first, start, dead_spots[i].second, end);
      for (unsigned int j = start; j <= end; ++j) {
	lcalib->set_distances(j, 0.0);
      }
    }
    lcalib->write();
    printf("Storing data in BlackBoard for visualization. Press Ctrl-C to quit.\n");
    while (1) {
      usleep(1000000);
    }
  }

  delete blackboard;
  delete client;

  return 0;
}
