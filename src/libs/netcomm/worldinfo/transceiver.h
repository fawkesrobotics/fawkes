
/***************************************************************************
 *  transceiver.h - World Info Transceiver
 *
 *  Created: Sun Jan 14 17:56:54 2007
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

#ifndef __NETCOMM_WORLDINFO_TRANSCEIVER_H_
#define __NETCOMM_WORLDINFO_TRANSCEIVER_H_

#include <core/exception.h>
#include <core/utils/lock_list.h>

#include <netcomm/worldinfo/handler.h>
#include <netcomm/worldinfo/defs.h>

#include <map>
#include <string>
#include <cstddef>
#include <ctime>

class MulticastDatagramSocket;
class WorldInfoMessageEncryptor;
class WorldInfoMessageDecryptor;
class NetworkNameResolver;

class WorldInfoException : public Exception
{
 public:
  WorldInfoException(const char *msg);
};

class WorldInfoTransceiver
{
 public:
  WorldInfoTransceiver(const char *addr, unsigned short port,
		       const char *key, const char *iv,
		       NetworkNameResolver *resolver = NULL);
  ~WorldInfoTransceiver();

  void add_handler(WorldInfoHandler *h);
  void rem_handler(WorldInfoHandler *h);

  void set_pose(float x, float y, float theta, float *covariance);
  void set_velocity(float vel_x, float vel_y, float vel_theta);

  void set_ball_pos(float dist, float pitch, float yaw, float *covariance);
  void set_ball_velocity(float vel_x, float vel_y, float vel_z);

  void clear_opponents();
  void add_opponent(float distance, float angle);

  void send();
  void recv(bool block = false, unsigned int max_num_msgs = 0);

  void set_loop(bool loop);
  void flush_sequence_numbers(unsigned int sec);

  void *  last_sent_plain_buffer();
  size_t  last_sent_plain_buffer_size();
  void *  last_sent_crypted_buffer();
  size_t  last_sent_crypted_buffer_size();

 private:
  void reset_outbound();
  void crypt_outbound();
  void append_outbound(uint16_t msg_type, void *msg, uint16_t msg_size);

  MulticastDatagramSocket *s;

  WorldInfoMessageEncryptor *encryptor;
  WorldInfoMessageDecryptor *decryptor;

  NetworkNameResolver       *resolver;
  bool                       resolver_delete;

  void  *in_buffer;
  void  *out_buffer;
  void  *crypted_out_buffer;
  void  *crypted_in_buffer;
  size_t crypt_buffer_size;

  size_t crypted_out_bytes;
  size_t crypted_in_bytes;

  unsigned int out_seq;
  unsigned int in_seq;

  unsigned char *outbound_buffer;
  unsigned int outbound_bytes;
  unsigned int outbound_num_msgs;

  unsigned char *inbound_buffer;
  size_t         inbound_bytes;

  float *covariance;

  bool   pose_changed;
  float  pose_x;
  float  pose_y;
  float  pose_theta;
  float *pose_covariance;

  bool   vel_changed;
  float  vel_x;
  float  vel_y;
  float  vel_theta;

  bool   ball_changed;
  float  ball_dist;
  float  ball_pitch;
  float  ball_yaw;
  float *ball_covariance;

  bool   ball_vel_changed;
  float  ball_vel_x;
  float  ball_vel_y;
  float  ball_vel_z;

  typedef struct {
    float distance;
    float angle;
  } opponent_t;
  std::list<opponent_t> opponents;
  std::list<opponent_t>::iterator oppit;

  LockList<WorldInfoHandler *> handlers;
  LockList<WorldInfoHandler *>::iterator hit;

  // Currently we only support IPv4
  std::map<uint32_t, unsigned int>       sequence_numbers;
  std::map<uint32_t, time_t>             last_received_time;
  std::map<uint32_t, time_t>::iterator   lrtit;
};


#endif
