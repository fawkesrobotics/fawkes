/***************************************************************************
 *  abstract_node_consstraints.h
 *
 *  Created: Sun Mar 02 10:47:35 2014
 *  Copyright  2014  Sebastian Reuter
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

#ifndef __NAVGRAPH_ABSTRACT_NODE_CONSTRAINT_H_
#define __NAVGRAPH_ABSTRACT_NODE_CONSTRAINT_H_

#include <logging/logger.h>
#include <config/config.h>
#include <vector>
#include <string>

#include <utils/graph/topological_map_graph.h>

namespace fawkes{

class AbstractNodeConstraint {

private:
	Logger *logger;

protected:
	std::vector<fawkes::TopologicalMapNode> nodeList;
	std::string constraintName;

public:
	/** Constructor
	 */
	AbstractNodeConstraint(Logger *logger, std::string name);

	/** Constructor
	 */
	AbstractNodeConstraint(Logger *logger, std::string name, std::vector<fawkes::TopologicalMapNode> nodeList);

	/** Destructor
	 */
	virtual ~AbstractNodeConstraint() = 0;

	/** Adds node to constraintList
	 */
	void add_node(fawkes::TopologicalMapNode node);

	/** Adds nodes to constraintList
	 */
	void add_nodes(std::vector<fawkes::TopologicalMapNode> nodes);


	/** removes node to from constraintList
	 */
	void remove_node(fawkes::TopologicalMapNode node);

	/** removes node to from constraintList
	*/
	bool has_node(fawkes::TopologicalMapNode node);

	/** returns name of constraint
	*/
	std::string get_constraint_name();

	/** returns const List of Nodes
	*/
	std::vector<fawkes::TopologicalMapNode> get_node_list() const;

	/** deletes all Nodes of list
	*/
	void clear_nodes();

};

/* ******************************************************************************************************** */
/* ************************************** IMPLEMENTATION DETAILS ****************************************** */
/* ******************************************************************************************************** */

inline AbstractNodeConstraint::AbstractNodeConstraint(Logger *logger, std::string name){
	this->logger = logger;
	this->constraintName = name;
}

inline AbstractNodeConstraint::AbstractNodeConstraint(Logger *logger, std::string name, std::vector<fawkes::TopologicalMapNode> nodeList){
	this->logger = logger;
	this->constraintName = name;
	this->nodeList = nodeList;
}

inline AbstractNodeConstraint::~AbstractNodeConstraint(){
}

inline void AbstractNodeConstraint::add_node(fawkes::TopologicalMapNode node){

	if( !this->has_node(node) ){
		this->nodeList.push_back( node );
		logger->log_info("abstract_node_constraint", "Added Node %s to '%s'.", node.name().c_str(), constraintName.c_str() );	}
	else{
		logger->log_info("abstract_node_constraint", "Node %s is already in '%s'", node.name().c_str(), constraintName.c_str() );
	}
}

inline void AbstractNodeConstraint::add_nodes(std::vector<fawkes::TopologicalMapNode> nodes){

	for(unsigned int i=0; i<nodes.size(); i++){
		logger->log_info("abstract_node_constraint", "Added Node %s to '%s'", nodes[i].name().c_str(), constraintName.c_str() );
		add_node( nodes[i] );
	}

}

inline void AbstractNodeConstraint::remove_node(fawkes::TopologicalMapNode node){

	if( ! this->nodeList.empty()){

		std::vector<TopologicalMapNode>::iterator i;
		for( i = this->nodeList.begin(); i != this->nodeList.end(); ++i){
			if( i->name() == node.name() ){
				this->nodeList.erase(i);
				logger->log_info("abstract_node_constraint", "removed node %s from constraint nodeList", node.name().c_str() );
			}
		}
	}

	logger->log_error("abstract_node_constraint", "Tried to remove not existing node %s from constraintList", node.name().c_str() );

}

inline bool AbstractNodeConstraint::has_node(fawkes::TopologicalMapNode node){

	if( !this->nodeList.empty() ){

		std::vector<TopologicalMapNode>::const_iterator i;
		for( i = this->nodeList.begin(); i != this->nodeList.end(); ++i){
			if( i->name() == node.name() ){
				return true;
			}
		}
	}
	return false;
}

inline std::string AbstractNodeConstraint::get_constraint_name(){
	return this->constraintName;
}

inline std::vector<fawkes::TopologicalMapNode> AbstractNodeConstraint::get_node_list() const{
	return this->nodeList;
}

inline void AbstractNodeConstraint::clear_nodes(){
	this->nodeList.clear();
}

} // fawkes



#endif
