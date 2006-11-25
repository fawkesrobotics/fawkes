
/***************************************************************************
 *  qa_socket_stream.cpp - Fawkes QA SeqPacketSocket
 *
 *  Created: Fri Nov 11 14:38:10 2006 (on train back from Google, Hamburg)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
#include <netcomm/socket/seqpacket.h>
#include <utils/system/signal.h>

#include <stdio.h>

class SeqPacketServerThread : public Thread
{
public:
  SeqPacketServerThread()
  {
    i = 0;
    try {
      s = new SeqPacketSocket();
      s->bind(1910);
      s->listen();
    } catch (SocketException &e) {
      e.printTrace();
      throw;
    }
    accepted = false;
    cs = NULL;
  }

  ~SeqPacketServerThread()
  {
    printf("Closing server socket\n");
    s->close();
    if (cs) cs->close();
    printf("Closed server socket\n");
    delete s;
    delete cs;
  }

  virtual void loop()
  {
    if (! accepted) {
      try {
	cs = s->accept();
	accepted = true;
      } catch (SocketException &e) {
	e.printTrace();
      }
    }
    if ( accepted ) {
      cs->write(&i, sizeof(i));
      unsigned int ri = 0;
      cs->read(&ri, sizeof(ri));
      if ( ri != i ) {
	printf("ERROR: sent %u but received %u\n", i, ri);
      } else {
	printf("OK: sent %u and received %u\n", i, ri);
      }
      ++i;
    }
  }

 private:
  unsigned int i;
  SeqPacketSocket *s;
  Socket *cs;
  bool accepted;
};


class SeqPacketClientThread : public Thread
{
public:
  SeqPacketClientThread()
  {
    s = new SeqPacketSocket();
    connected = false;
  }

  ~SeqPacketClientThread()
  {
    printf("Closing client socket\n");
    s->close();
    printf("Closed client socket\n");
    delete s;
  }

  virtual void loop()
  {
    if (! connected) {
      try {
	s->connect("127.0.0.1", 1910);
	connected = true;
      } catch (SocketException &e) {
	e.printTrace();
      }
    }
    if ( connected ) {
      unsigned int i = 0;
      s->read(&i, sizeof(i));
      s->write(&i, sizeof(i));
    }
  }

 private:
  SeqPacketSocket *s;
  bool connected;
};


class SeqPacketSocketQAMain : public SignalHandler
{
 public:
  SeqPacketSocketQAMain()
  {
    s = new SeqPacketServerThread();
    c = new SeqPacketClientThread();
  }

  ~SeqPacketSocketQAMain()
  {
    delete s;
    delete c;
  }


  virtual void handle_signal(int signum)
  {
    printf("Signal received, cancelling threads\n");
    s->cancel();
    c->cancel();
    printf("Threads cancelled\n");
  }

  void run()
  {
    s->start();
    c->start();
    s->join();
    c->join();
  }

 private:
  SeqPacketServerThread *s;
  SeqPacketClientThread *c;

};

int
main(int argc, char **argv)
{
  SeqPacketSocketQAMain m;
  SignalManager::register_handler(SIGINT, &m);
  SignalManager::ignore(SIGPIPE);

  m.run();
}

/// @endcond
