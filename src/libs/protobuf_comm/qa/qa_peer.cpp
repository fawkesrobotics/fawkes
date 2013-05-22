
/***************************************************************************
 *  qa_peer.cpp - protobuf_comm broadcast peer test program
 *
 *  Created: Mon Feb 04 18:25:41 2013
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

#include <protobuf_comm/peer.h>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <msgs/Person.pb.h>

using namespace protobuf_comm;
using namespace llsf_msgs;

/// @cond QA

static bool quit = false;
ProtobufBroadcastPeer *peer;

void
signal_handler(const boost::system::error_code& error, int signum)
{
  if (!error) {
    quit = true;
  }
}


void
handle_error(const boost::system::error_code &error)
{
  printf("Error: %s\n", error.message().c_str());
}

void
handle_message(boost::asio::ip::udp::endpoint &sender,
	       uint16_t component_id, uint16_t msg_type,
	       std::shared_ptr<google::protobuf::Message> msg)
{
  printf("Received message of type %u from %s\n", msg_type,
	 sender.address().to_string().c_str());
  std::shared_ptr<Person> p;
  if ((p = std::dynamic_pointer_cast<Person>(msg))) {
    printf("Person %i: %s <%s>\n", p->id(), p->name().c_str(), p->email().c_str());
  }

  //server.send(client, component_id, msg_type, *p);
}

int
main(int argc, char **argv)
{
  unsigned short send_to_port = 1234;
  unsigned short recv_on_port = 1234;
  if (argc >= 3) {
    send_to_port = boost::lexical_cast<unsigned short>(argv[1]);
    recv_on_port = boost::lexical_cast<unsigned short>(argv[2]);
  }
  peer = new ProtobufBroadcastPeer("192.168.0.255", send_to_port, recv_on_port);

  boost::asio::io_service io_service;

  MessageRegister & message_register = peer->message_register();
  message_register.add_message_type<Person>(1, 2);

  peer->signal_received().connect(handle_message);
  peer->signal_error().connect(handle_error);

  // Construct a signal set registered for process termination.
  boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);

  // Start an asynchronous wait for one of the signals to occur.
  signals.async_wait(signal_handler);

  if (argc >= 4) {
    Person p;
    p.set_id(1);
    p.set_name("Tim Niemueller");
    p.set_email("niemueller@kbsg.rwth-aachen.de");
    peer->send(1, 2, p);
  }

  do {
    io_service.run();
    io_service.reset();
  } while (! quit);

  delete peer;

  // Delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}

/// @endcond
