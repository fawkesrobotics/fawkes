
/***************************************************************************
 *  battery_plugin.cpp - Fawkes Battery Plugin
 *
 *  Generated: Tue Jan 29 13:05:15 2008
 *  Copyright  2008  Daniel Beck
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/battery/battery_thread.h>
#include <interfaces/battery.h>
#include <utils/time/wait.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/** @class BatteryThread <plugins/battery/battery_thread.h>
 * This is the main thread of the battery plugin. It reads out some values
 * from the battery and writes them to the blackboard.
 *
 * @author Daniel Beck
 */

/** Constructor. */
BatteryThread::BatteryThread()
  : Thread("BatteryThread", Thread::OPMODE_CONTINUOUS)
{
  m_port = 0;
  set_prepfin_conc_loop(true);
}

/** Destructor. */
BatteryThread::~BatteryThread()
{
  free(m_port);
}

void
BatteryThread::finalize()
{
  try
    {
      blackboard->close(m_battery_interface);
    }
  catch (Exception& e)
    {
      logger->log_error(name(), "Could not close battery interface");
      logger->log_error(name(), e);
    }

  // restore settings
  tcsetattr(m_fd, TCSANOW, &m_old_options);
  close(m_fd);
}

void
BatteryThread::init()
{
  try
    {
      m_battery_interface = blackboard->open_for_writing<BatteryInterface>("Battery");
    }
  catch (Exception& e)
    {
      e.append("Opening battery interface failed");
      throw;
    }

  try
    {
      m_port = strdup(config->get_string("/battery/port").c_str());
      m_interval = config->get_uint("/battery/interval");
    }
  catch (Exception& e)
    {
      e.append("Configuration value not found");
      blackboard->close(m_battery_interface);
      throw;
    }
  
  // open serial port
  m_fd = open(m_port, O_RDWR | O_NOCTTY);

  if ( -1 == m_fd )
    {
      blackboard->close(m_battery_interface);
      throw Exception("Opening %s failed", m_port);
    }
  
  struct termios options;
  tcgetattr(m_fd, &m_old_options);
  memset(&options, 0, sizeof(options));
  options.c_cflag = CS8 | CREAD | CLOCAL | B9600;
  options.c_iflag = IGNBRK;
  options.c_oflag = 0;
  tcsetattr(m_fd, TCSANOW, &options);

  m_time_wait = new TimeWait(clock, m_interval * 1000000);
}

void
BatteryThread::loop()
{
  m_time_wait->mark_start();

  unsigned int current;
  unsigned int voltage;
  unsigned int temperature;

  // current
  current = 65535 - read_numeric(0x1d);
  m_battery_interface->set_current(current);

  // voltage
  voltage = read_numeric(0x1e);
  m_battery_interface->set_voltage(voltage);

  // temperature
  temperature = read_numeric(0x1f);
  m_battery_interface->set_temperature(temperature);

  m_battery_interface->write();

  logger->log_debug(name(), "Current=%d  Voltage=%d  Temperature=%d",
		    current, voltage, temperature);

  // take a nap...
  m_time_wait->wait();
}

/** Send the given command to the battery.
 *
 * cmd       description
 * 0x00      Sleep
 * 0x01      Push button
 *
 * @return false if something failed, true otherwise
 */
/*
bool
BatteryThread::send_command(unsigned char cmd)
{
  return true;
}
*/

/** Reads out a specified field an returns the contents as numeric value.
 *
 * cmd       description
 * 0x1A      State of health
 * 0x1B      Relative SOC
 * 0x1C      Absolute SOC
 * 0x1D      Battery current
 * 0x1E      Battery voltage
 * 0x1F      Battery temperature
 * 0x20      Balancing time
 *
 * @return the contents of the field as numeric value
 */
unsigned int
BatteryThread::read_numeric(unsigned char cmd)
{
  ssize_t num_bytes;
  unsigned char buffer[255];
  unsigned int base;
  unsigned int ret_val = 0;

  num_bytes = write(m_fd, &cmd, 1);
  if ( 1 != num_bytes ) 
    { logger->log_warn(name(), "Sending command to battery failed"); }
  else
    {
      usleep(10000);
      
      memset(buffer, 0, 255);
      num_bytes = read(m_fd, buffer, 255);
      if ( 0 > num_bytes ) 
	{ logger->log_warn(name(), "Receiving data from battery failed"); }
      else
	{
	  base = 1;
	  for (unsigned int i = num_bytes - 1; i > 0; --i) 
	    { 
	      char c[4];
	      memset(c, 0, 4);
	      unsigned int t;
	      sprintf(c, "%d", buffer[i]);
	      t = atoi(c);
	      ret_val += base * t;
	      base *= 256;
	    }
	}
    }

  return ret_val;
}

/*
unsigned int
BatteryThread::read_string(unsigned char cmd)
{
}
*/
