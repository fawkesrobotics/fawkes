
/***************************************************************************
 *  battery_monitor_treeview.cpp - TreeView class for displaying the battery
 *                                 status of the robots
 *
 *  Created: Mon Apr 06 16:08:50 2009
 *  Copyright  2009  Daniel Beck
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

#include "battery_monitor_treeview.h"

#include <blackboard/remote.h>
#include <gui_utils/interface_dispatcher.h>
#include <interfaces/BatteryInterface.h>

#include <cstring>

using namespace std;
using namespace fawkes;

/** @class BatteryMonitorTreeView tools/battery_monitor/battery_monitor_treeview.h
 * A treeview that retrieves battery data from the robots over remote
 * blackboard connections and displays those.
 * @author Daniel Beck
 */

/** @class BatteryMonitorTreeView::BatteryRecord tools/battery_monitor/battery_monitor_treeview.h
 * Column record class for the battery monitor treeview.
 * @author Daniel Beck
 */

/** @var BatteryMonitorTreeView::m_battery_record
 * Column record object to acces the columns of the storage object.
 */

/** @var BatteryMonitorTreeView::m_battery_list
 * Storage object.
 */

/** @var BatteryMonitorTreeView::m_remote_bbs
 * Map with remote blackboards: hostname -> remote blackboard.
 */

/** @var BatteryMonitorTreeView::m_battery_interfaces
 * Map containing the battery interfaces: hostname -> battery interface
 */

/** @var BatteryMonitorTreeView::m_interface_dispatcher
 * Interface dispatcher for the battery interfaces.
 */

/** Constructor.
 * @param cobject base object type
 * @param builder builder to get widgets from
 */
BatteryMonitorTreeView::BatteryMonitorTreeView(BaseObjectType* cobject,
                                               const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::TreeView( cobject )
{
  m_battery_list = Gtk::ListStore::create( m_battery_record );
  set_model( m_battery_list );

  append_column( "Host", m_battery_record.short_name );
  append_column_numeric( "Abs. SOC [%]", m_battery_record.absolute_soc, "%.1f" ); 
  append_column_numeric( "Rel. SOC [%]", m_battery_record.relative_soc, "%.1f" ); 
  append_column_numeric( "Voltage [V]", m_battery_record.voltage, "%.3f" );
  append_column_numeric( "Current [A]", m_battery_record.current, "%.3f" );

  builder->get_widget("dlgWarning", m_dlg_warning);
  m_dlg_warning->hide();

  m_trigger_update.connect(sigc::mem_fun(*this, &BatteryMonitorTreeView::update));

  m_relative_soc_threshold = 20.0;
}

/** Destructor. */
BatteryMonitorTreeView::~BatteryMonitorTreeView()
{
  std::map< string, BatteryInterface* >::iterator biit;
  for (biit = m_battery_interfaces.begin();
       biit != m_battery_interfaces.end();
       ++biit)
  {
    std::map< string, BlackBoard* >::iterator rbit;
    rbit = m_remote_bbs.find( biit->first );
    
    std::map< string, InterfaceDispatcher* >::iterator idit;
    idit = m_interface_dispatcher.find( biit->first );

    if ( rbit != m_remote_bbs.end() )
    {
      rbit->second->unregister_listener( idit->second );
      rbit->second->close( biit->second );
      delete rbit->second;
    }
  }

  // delete interface dispatcher
  std::map< string, InterfaceDispatcher* >::iterator i;
  for (i = m_interface_dispatcher.begin();
       i != m_interface_dispatcher.end();
       ++i )
  {
    delete i->second;
  }

  // delete remote blackboards
  for ( std::map< string, BlackBoard* >::iterator i = m_remote_bbs.begin();
	i != m_remote_bbs.end();
	++i )
  {
    delete i->second;
  }

  delete m_dlg_warning;
}

/** Add given host.
 * @param h the host's hostname
 */
void
BatteryMonitorTreeView::add_host( const char* h )
{
  string host(h);

  BlackBoard* rbb;
  std::map< string, BlackBoard* >::iterator i = m_remote_bbs.find( host );

  if ( i == m_remote_bbs.end() )
    // no remote blackboard opened, yet
  {
    try
    { 
      rbb = new RemoteBlackBoard( h, 1910 );
      m_remote_bbs[ host ] = rbb;
    }
    catch ( Exception& e )
    {
      e.append( "Could not open remote blackboard on host %s", h );
      e.print_trace();
      return;
    }
  }
  else
  { rbb = i->second; }

  if ( m_battery_interfaces.find( host ) == m_battery_interfaces.end() )
    // no battery interface opened, yet
  {
    try
    {
      BatteryInterface* bi;
      bi = rbb->open_for_reading< BatteryInterface >( "Battery" );
      m_battery_interfaces[ host ] = bi;

      InterfaceDispatcher* id =
        new InterfaceDispatcher( "BatteryMonitorTreeView", bi );

      id->signal_data_changed().connect( sigc::mem_fun( *this,
							&BatteryMonitorTreeView::on_data_changed ) );
      id->signal_writer_added().connect( sigc::mem_fun( *this,
							&BatteryMonitorTreeView::on_writer_added ) );
      id->signal_writer_removed().connect( sigc::mem_fun( *this,
							  &BatteryMonitorTreeView::on_writer_removed ) );
      rbb->register_listener( id, BlackBoard::BBIL_FLAG_DATA | BlackBoard::BBIL_FLAG_WRITER );
    }
    catch ( Exception& e )
    {
      e.append( "Opening battery interface on host %s failed", h );
      e.print_trace();
    }

    // add below threshold counter
    m_below_threshold_counter[ host ] = 0;
  }
  
  m_trigger_update();
}

/** Remove given host.
 * @param h the host's hostname
 */
void
BatteryMonitorTreeView::rem_host( const char* h )
{
  string host( h );

  std::map< string, BlackBoard* >::iterator rbbit = m_remote_bbs.find( host );
  if ( m_remote_bbs.end() == rbbit )
    // no blackboard opened---nothing to do
  { return; }

  std::map< string, BatteryInterface* >::iterator biit = m_battery_interfaces.find( host );

  if ( m_battery_interfaces.end() != biit )
    // battery inteface opened. listener need to be unregistered and
    // interface nees to be closed
  {
    try
    {
      BlackBoard* rbb = rbbit->second;
      InterfaceDispatcher* id = m_interface_dispatcher.find( host )->second;
      rbb->unregister_listener( id );
      rbb->close( biit->second );
      m_battery_interfaces.erase( biit );
    }
    catch ( Exception& e )
    {
      e.append( "Closing battery interface for host %s could not be closed", h );
      e.print_trace();
    }
  }

  // destroy blackboard
  delete rbbit->second;
  m_remote_bbs.erase( rbbit );

  // remove below threshold counter
  m_below_threshold_counter.erase( host );

  m_trigger_update();
}

void
BatteryMonitorTreeView::update()
{
  // clear treeview
  Gtk::TreeModel::Children::iterator rit = m_battery_list->children().begin();
  while ( rit != m_battery_list->children().end() )
  {
    rit = m_battery_list->erase( rit );
  }

  for ( std::map< string, BatteryInterface* >::iterator biit = m_battery_interfaces.begin();
	biit != m_battery_interfaces.end();
	++biit )
  {
    // update data in interface
    BatteryInterface* bi = biit->second;

    try
    {
      bi->read();
    }
    catch ( Exception& e )
    {
      e.append( "read() failed" );
      e.print_trace();
      continue;
    }

    if ( !bi->has_writer() )
      // only consider interfaces which have a writer
    { continue; }

    Gtk::TreeModel::Row row;
    row = *m_battery_list->append();
    row[ m_battery_record.fqdn ] = Glib::ustring( biit->first );
    
    char* fqdn = strdup( (biit->first).c_str() );
    char* sh;
    char delim = '.';
    sh = strtok( fqdn, &delim );
    int i = atoi( sh );
    
    if ( 0 != i )
    { row[ m_battery_record.short_name ] = Glib::ustring( biit->first ); }
    else
    { row[ m_battery_record.short_name ] = Glib::ustring( sh ); }
    
    row[ m_battery_record.absolute_soc ] = bi->absolute_soc() * 100.0;
    row[ m_battery_record.relative_soc ] = bi->relative_soc() * 100.0;
    row[ m_battery_record.current ] = bi->current() / 1000.0;
    row[ m_battery_record.voltage ] = bi->voltage() / 1000.0;

    string fqdn_str = string( fqdn );
    if ( row[ m_battery_record.relative_soc ] <= m_relative_soc_threshold )
    {
      unsigned int cnt = m_below_threshold_counter[ fqdn_str ];
      m_below_threshold_counter[ fqdn_str ] = ++cnt;
    }
    else
    { m_below_threshold_counter[ fqdn_str ] = 0; }

    free(fqdn);
  }
  
  Glib::ustring secondary = "The batteries on ";
  bool below_threshold = false;

  for ( std::map< string, unsigned int >::iterator i = m_below_threshold_counter.begin();
	i != m_below_threshold_counter.end();
	++i )
  {
    if ( i->second > 2 )
    {
      secondary += "<b>" + Glib::ustring( (i->first).c_str() ) + "</b>" + " ";
      i->second = 0;

      below_threshold = true;
    }
  }
  secondary += "need to be replaced.";

  if ( below_threshold )
  {
    m_dlg_warning->set_secondary_text( secondary, true );
    m_dlg_warning->set_urgency_hint();
    m_dlg_warning->run();
    m_dlg_warning->hide();
  }
}

void
BatteryMonitorTreeView::on_data_changed( fawkes::Interface* interface )
{
  update();
}

void
BatteryMonitorTreeView::on_writer_added( fawkes::Interface* interface )
{
  update();
}

void
BatteryMonitorTreeView::on_writer_removed( fawkes::Interface* interface )
{
  update();
}
