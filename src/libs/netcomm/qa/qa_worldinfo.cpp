
/***************************************************************************
 *  qa_worldinfo.cpp - Fawkes QA WorldInfoTransceiver
 *
 *  Created: Thu May 03 16:14:59 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

/// @cond QA

#include <core/threading/thread.h>
#include <netcomm/worldinfo/transceiver.h>
#include <utils/system/signal.h>
#include <utils/system/argparser.h>

#include <netdb.h>
#include <cstdio>
#include <cstring>
#include <iostream>

using namespace std;

class WorldInfoSenderThread : public Thread
{
public:
  WorldInfoSenderThread(unsigned short int port, bool loop)
  {
    i = 0;
    try {
      t = new WorldInfoTransceiver("224.16.0.1", port,
				   "AllemaniACs", "WorldInfoQA");
      t->set_loop( loop );
    } catch (Exception &e) {
      e.printTrace();
      throw;
    }
    covariance = (float *)malloc(WORLDINFO_COVARIANCE_SIZE * sizeof(float));
    for (unsigned int j = 0; j < WORLDINFO_COVARIANCE_SIZE; ++j) {
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
    t->set_velocity(i+3, i+4, i+5);
    t->set_ball_pos(i+6, i+7, i+8, covariance);
    t->set_ball_velocity(i+9, i+10, i+11);
    t->add_opponent(i+12, i+13);
    t->add_opponent(i+14, i+15);
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
  WorldInfoReceiverThread(unsigned short int port, unsigned int max_num_msgs)
  {
    this->max_num_msgs = max_num_msgs;
    try {
      t = new WorldInfoTransceiver("224.16.0.1", port,
				   "AllemaniACs", "WorldInfoQA");
      t->add_handler(this);
    } catch (Exception &e) {
      e.printTrace();
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
    t->recv( /* block = */ true, max_num_msgs );
  }

  virtual void pose_rcvd(const char *from_host,
			 float x, float y, float theta,
			 float *covariance)
  {
    cout << "Pose[" << from_host << "]: (x,y,th)=("
	 << x << "," << y << "," << theta << "), cov=(";
    for ( unsigned int i = 0; i < WORLDINFO_COVARIANCE_SIZE; ++i) {
      cout << covariance[i];
      if ( i != WORLDINFO_COVARIANCE_SIZE-1 ) {
	cout << ",";
      }
    }
    cout << ")" << endl;
  }

  virtual void velocity_rcvd(const char *from_host, float vel_x,
			     float vel_y, float vel_theta)
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
    for ( unsigned int i = 0; i < WORLDINFO_COVARIANCE_SIZE; ++i) {
      cout << covariance[i];
      if ( i != WORLDINFO_COVARIANCE_SIZE-1 ) {
	cout << ",";
      }
    }
    cout << ")" << endl;
  }

  virtual void ball_velocity_rcvd(const char *from_host,
				  float vel_x, float vel_y, float vel_z)
  {
    cout << "BVel[" << from_host << "]: (vx,vy,vz)=("
	 << vel_x << "," << vel_y << "," << vel_z << ")" << endl;
  }

  virtual void opponent_pose_rcvd(const char *from_host,
			     float distance, float angle)
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
    s = NULL;
    r = NULL;
    this->argp = argp;
    if ( argp->hasArgument("r") ) {
      printf("Going to be a receiver\n");
      r = new WorldInfoReceiverThread(1910, argp->hasArgument("s") ? 1 : 0);
    } else {
      s = new WorldInfoSenderThread(1910, argp->hasArgument("l"));
    }
  }

  ~WorldInfoQAMain()
  {
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
};

int
main(int argc, char **argv)
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "rlsH");

  if ( argp->hasArgument("H") ) {
    cout << "Usage: " << argv[0] << "[-r] [-H] [-s] [-l]" << endl
	 << " -r   receiver (sender otherwise)" << endl
	 << " -H   this help message" << endl
	 << " -s   single per recv, only process a single message per recv()" << endl
	 << " -l   enable multicast loop back" << endl;
    return 0;
  }

  WorldInfoQAMain m(argp);
  SignalManager::register_handler(SIGINT, &m);
  SignalManager::ignore(SIGPIPE);

  m.run();

  delete argp;
  return 0;
}

/// @endcond
