
/***************************************************************************
 *  skilltester.cpp - Skilltester for eclipse-clp - console tool
 *
 *  Created: Tue Apr 09 12:36:39 2013
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
               2013       Gesche Gierse
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

#include <blackboard/remote.h>
#include <core/threading/thread.h>
#include <interfaces/TestInterface.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/client_handler.h>
#include <readline/history.h>
#include <readline/readline.h>
#include <utils/system/argparser.h>
#include <utils/system/signal.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

using namespace fawkes;

void
print_usage(const char *program_name)
{
	printf("Usage: %s [-h] [-r host[:port]]\n"
	       " -h              This help message\n"
	       " -r host[:port]  Remote host (and optionally port) to connect to\n",
	       program_name);
}

static int
event_hook()
{
	return 0;
}

/** Skill shell thread.
 * This thread opens a network connection to a host and uses a RemoteBlackBoard
 * connection to send skill strings for execution. It also shows Skiller log messages
 * and uses the skiller network protocol.
 * @author Tim Niemueller
 */
class SkillShellThread : public Thread, public FawkesNetworkClientHandler
{
public:
	/** Constructor.
   * @param argp argument parser
   */
	explicit SkillShellThread(ArgumentParser *argp)
	: Thread("SkillShellThread", Thread::OPMODE_CONTINUOUS)
	{
		this->argp               = argp;
		prompt                   = "-# ";
		just_connected           = true;
		connection_died_recently = false;

		testif = NULL;
		using_history();
		// this is needed to get rl_done working
		rl_event_hook = event_hook;

		char              *host      = (char *)"localhost";
		unsigned short int port      = 1910;
		bool               free_host = argp->parse_hostport("r", &host, &port);

		c = new FawkesNetworkClient(host, port);

		if (free_host)
			free(host);

		c->register_handler(this, FAWKES_CID_SKILLER_PLUGIN);
		c->connect();
	}

	/** Destructor. */
	~SkillShellThread()
	{
		printf("Finalizing\n");

		//SkillerInterface::ReleaseControlMessage *rcm = new SkillerInterface::ReleaseControlMessage();
		//sif->msgq_enqueue(rcm);

		usleep(500000);

		rbb->close(testif);
		delete rbb;
		rbb = NULL;

		c->deregister_handler(FAWKES_CID_SKILLER_PLUGIN);
		c->disconnect();
		delete c;
	}

	virtual void
	loop()
	{
		if (c->connected()) {
			if (just_connected) {
				just_connected = false;
				try {
					rbb    = new RemoteBlackBoard(c);
					testif = rbb->open_for_reading<TestInterface>("eclipse_clp_skillexec");
					usleep(100000);
				} catch (Exception &e) {
					e.print_trace();
					return;
				}
			}

			if (argp->num_items() > 0) {
				const std::vector<const char *> &items = argp->items();

				std::vector<const char *>::const_iterator i   = items.begin();
				std::string                               sks = *i;
				++i;
				for (; i != items.end(); ++i) {
					sks += " ";
					sks += *i;
				}

				TestInterface::SetTestStringMessage *tsm =
				  new TestInterface::SetTestStringMessage(sks.c_str());
				testif->msgq_enqueue(tsm);

				usleep(100000);
				exit();
			} else {
				char *line = readline(prompt);
				if (line) {
					if (strcmp(line, "") != 0) {
						TestInterface::SetTestStringMessage *tsm =
						  new TestInterface::SetTestStringMessage(line);
						testif->msgq_enqueue(tsm);
						add_history(line);
					}
				} else {
					if (!connection_died_recently) {
						exit();
					}
				}
			}
		} else {
			if (connection_died_recently) {
				connection_died_recently = false;
				printf("Connection died\n");
				c->disconnect();
			}
			try {
				c->connect();
			} catch (Exception &e) {
				printf(".");
				fflush(stdout);
				sleep(1);
			}
		}
	}

	virtual void
	deregistered(unsigned int id) noexcept
	{
	}

	virtual void
	inbound_received(FawkesNetworkMessage *m, unsigned int id) noexcept
	{
	}

	virtual void
	connection_died(unsigned int id) noexcept
	{
		prompt = "-# ";

		rbb->close(testif);
		delete rbb;
		rbb    = NULL;
		testif = NULL;

		connection_died_recently = true;

		//fprintf(stdin, "\n");
		//kill(SIGINT);
		rl_done = 1;
	}

	virtual void
	connection_established(unsigned int id) noexcept
	{
		printf("Connection established\n");
		just_connected = true;
		prompt         = "+# ";
	}

private:
	ArgumentParser      *argp;
	FawkesNetworkClient *c;
	BlackBoard          *rbb;
	TestInterface       *testif;
	const char          *prompt;
	bool                 just_connected;
	bool                 connection_died_recently;
};

/** Config tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
	ArgumentParser argp(argc, argv, "hr:");

	if (argp.has_arg("h")) {
		print_usage(argv[0]);
		exit(0);
	}

	SkillShellThread sst(&argp);
	sst.start();
	sst.join();

	return 0;
}
