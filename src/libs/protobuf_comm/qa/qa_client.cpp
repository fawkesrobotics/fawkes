
/***************************************************************************
 *  qa_client.cpp - protobuf_comm client test program
 *
 *  Created: Thu Jan 31 22:00:44 2013
 *  Copyright  2013  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/asio.hpp>
#include <protobuf_comm/client.h>

#include <msgs/MachineInfo.pb.h>

using namespace protobuf_comm;
using namespace llsf_msgs;

/// @cond QA

static bool quit = false;
static ProtobufStreamClient client;

void
signal_handler(const boost::system::error_code& error, int signum)
{
  if (!error) {
    quit = true;
  }
}


void
connected()
{
  /*
  Person p;
  p.set_id(1);
  p.set_name("Tim Niemueller");
  p.set_email("niemueller@kbsg.rwth-aachen.de");
  client.send(1, 2, p);
  */
}

void
handle_message(uint16_t comp_id, uint16_t msg_type,
	       std::shared_ptr<google::protobuf::Message> msg)
{
  printf("Received message of type %u\n", msg_type);
  /*
  std::shared_ptr<Person> p;
  if ((p = std::dynamic_pointer_cast<Person>(msg))) {
    printf("Person %i: %s <%s>\n", p->id(), p->name().c_str(), p->email().c_str());
  }
  */
}


int
main(int argc, char **argv)
{
  boost::asio::io_service io_service;

  boost::asio::deadline_timer  timer_(io_service);
  boost::asio::deadline_timer  reconnect_timer_(io_service);
  boost::asio::deadline_timer  attmsg_timer_(io_service);
  boost::asio::deadline_timer  blink_timer_(io_service);

  client.signal_connected().connect(connected);
  client.async_connect("127.0.0.1", 4444);

  //MessageRegister & message_register = client.message_register();
  //message_register.add_message_type<Person>(1, 2);

  client.signal_received().connect(handle_message);

  // Construct a signal set registered for process termination.
  boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);

  // Start an asynchronous wait for one of the signals to occur.
  signals.async_wait(signal_handler);

  do {
    io_service.run();
    io_service.reset();
  } while (! quit);

  // Delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}

/// @endcond
