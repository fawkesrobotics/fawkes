
/***************************************************************************
 *  qa_worldinfo.cpp - Fawkes QA WorldInfoTransceiver
 *
 *  Created: Thu May 03 16:14:59 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

/// @cond QA

#include <core/threading/thread.h>
#include <netcomm/worldinfo/transceiver.h>
#ifdef HAVE_AVAHI
#include <netcomm/dns-sd/avahi_thread.h>
#endif
#include <netcomm/utils/resolver.h>
#include <utils/system/signal.h>
#include <utils/system/argparser.h>

#include <netdb.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>

using namespace std;

class WorldInfoSenderThread : public Thread
{
public:
  WorldInfoSenderThread(unsigned short int port, bool loop, NetworkNameResolver *rs)
    : Thread("WorldInfoSenderThread", Thread::OPMODE_CONTINUOUS)
  {
    i = 0;
    try {
      t = new WorldInfoTransceiver("224.16.0.1", port,
				   "AllemaniACs", "WorldInfoQA",
				   rs);
      t->set_loop( loop );
    } catch (Exception &e) {
      e.print_trace();
      throw;
    }
    covariance = (float *)malloc(WORLDINFO_COVARIANCE_SIZE_3X3 * sizeof(float));
    for (unsigned int j = 0; j < WORLDINFO_COVARIANCE_SIZE_3X3; ++j) {
      covariance[j] = j;
    }
  }

  ~WorldInfoSenderThread()
  {
    printf("Closing sender\n");
    delete t;
    free(covariance);
  }

  virtual void loop()
  {
    printf("Sending %u\n", i);
    t->set_pose(i, i+1, i+2, covariance);
    t->set_velocity(i+3, i+4, i+5, covariance);
    t->set_ball_pos(i+6, i+7, i+8, covariance);
    t->set_ball_velocity(i+9, i+10, i+11, covariance);
    t->add_opponent(i+12, i+13, covariance);
    t->add_opponent(i+14, i+15, covariance);
    t->send();
    ++i;
  }

 private:
  WorldInfoTransceiver *t;
  unsigned int i;
  float *covariance;
};


class WorldInfoReceiverThread : public Thread, public WorldInfoHandler
{
public:
  WorldInfoReceiverThread(unsigned short int port, unsigned int max_num_msgs,
			  NetworkNameResolver *rs)
    : Thread("WorldInfoReceiverThread", Thread::OPMODE_CONTINUOUS)
  {
    this->max_num_msgs = max_num_msgs;
    try {
      t = new WorldInfoTransceiver("224.16.0.1", port,
				   "AllemaniACs", "WorldInfoQA",
				   rs);
      t->add_handler(this);
    } catch (Exception &e) {
      e.print_trace();
      throw;
    }
  }

  ~WorldInfoReceiverThread()
  {
    printf("Closing receiver\n");
    delete t;
  }

  virtual void loop()
  {
    printf("Waiting for data\n");
    t->flush_sequence_numbers(10);
    t->recv( /* block = */ true, max_num_msgs );
  }

  virtual void pose_rcvd(const char *from_host,
			 float x, float y, float theta,
			 float *covariance)
  {
    cout << "Pose[" << from_host << "]: (x,y,th)=("
	 << x << "," << y << "," << theta << "), cov=(";
    for ( unsigned int i = 0; i < WORLDINFO_COVARIANCE_SIZE_3X3; ++i) {
      cout << covariance[i];
      if ( i != WORLDINFO_COVARIANCE_SIZE_3X3 - 1 ) {
	cout << ",";
      }
    }
    cout << ")" << endl;
  }

  virtual void velocity_rcvd(const char *from_host, float vel_x,
			     float vel_y, float vel_theta, float *covariance)
  {
    cout << "Velo[" << from_host << "]: (vx,vy,vth)=("
	 << vel_x << "," << vel_y << "," << vel_theta << ")" << endl;
  }

  virtual void ball_pos_rcvd(const char *from_host,
			     float dist, float pitch, float yaw,
			     float *covariance)
  {
    cout << "Ball[" << from_host << "]: (d,p,y)=("
	 << dist << "," << pitch << "," << yaw << "), cov=(";
    for ( unsigned int i = 0; i < WORLDINFO_COVARIANCE_SIZE_3X3; ++i) {
      cout << covariance[i];
      if ( i != WORLDINFO_COVARIANCE_SIZE_3X3 - 1 ) {
	cout << ",";
      }
    }
    cout << ")" << endl;
  }

  virtual void ball_velocity_rcvd(const char *from_host,
				  float vel_x, float vel_y, float vel_z, float *covariance)
  {
    cout << "BVel[" << from_host << "]: (vx,vy,vz)=("
	 << vel_x << "," << vel_y << "," << vel_z << ")" << endl;
  }

  virtual void opponent_pose_rcvd(const char *from_host,
			     float distance, float angle, float *covariance)
  {
    cout << "Oppt[" << from_host << "]: (d,a)=("
	 << distance << "," << angle << ")" << endl;
  }


 private:
  WorldInfoTransceiver *t;
  unsigned int max_num_msgs;
};


class WorldInfoQAMain : public SignalHandler
{
 public:
  WorldInfoQAMain(ArgumentParser *argp)
  {
#ifdef HAVE_AVAHI
    if ( argp->has_arg("a") ) {
      at = new AvahiThread();
      at->start();
      printf("Waiting for Avahi thread to initialize\n");
      at->wait_initialized();
    } else {
      at = NULL;
    }
    rs = new NetworkNameResolver(at);
#else
    rs = new NetworkNameResolver();
#endif
    s = NULL;
    r = NULL;
    this->argp = argp;
    if ( argp->has_arg("r") ) {
      printf("Going to be a receiver\n");
      r = new WorldInfoReceiverThread(1910, argp->has_arg("s") ? 1 : 0, rs);
    } else {
      s = new WorldInfoSenderThread(1910, argp->has_arg("l"), rs);
    }
  }

  ~WorldInfoQAMain()
  {
#ifdef HAVE_AVAHI
    if ( at != NULL ) {
      at->cancel();
      at->join();
      delete at;
    }
#endif
    delete s;
    delete r;
  }


  virtual void handle_signal(int signum)
  {
    printf("Signal received, cancelling threads\n");
    if ( s != NULL )  s->cancel();
    if ( r != NULL )  r->cancel();
    printf("Threads cancelled\n");
  }

  void run()
  {
    if ( s != NULL ) {
      s->start();
      s->join();
    }
    if ( r != NULL ) {
      r->start();
      r->join();
    }
  }

 private:
  ArgumentParser *argp;
  WorldInfoSenderThread *s;
  WorldInfoReceiverThread *r;
  NetworkNameResolver *rs;
#ifdef HAVE_AVAHI
  AvahiThread *at;
#endif
};

int
main(int argc, char **argv)
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "arlsh");

  if ( argp->has_arg("h") ) {
    cout << "Usage: " << argv[0] << "[-r] [-h] [-s] [-l] [-a]" << endl
	 << " -r   receiver (sender otherwise)" << endl
	 << " -h   this help message" << endl
	 << " -s   single per recv, only process a single message per recv()" << endl
#ifdef HAVE_AVAHI
	 << " -a   enable Avahi for mDNS lookup" << endl
#else
	 << " -a   not available (Avahi not installed)" << endl
#endif
	 << " -l   enable multicast loop back" << endl;
    return 0;
  }

  WorldInfoQAMain m(argp);
  SignalManager::register_handler(SIGINT, &m);
  SignalManager::ignore(SIGPIPE);

  m.run();

  SignalManager::finalize();

  delete argp;
  return 0;
}

/// @endcond
