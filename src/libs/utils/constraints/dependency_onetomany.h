
/***************************************************************************
 *  dependency_onetomany.h - One-to-Many dependency constraint
 *
 *  Created: Tue May 29 14:10:25 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __UTILS_CONSTRAINTS_DEPENDENCY_ONETOMANY_H_
#define __UTILS_CONSTRAINTS_DEPENDENCY_ONETOMANY_H_

#include <utils/constraints/dependency.h>

#include <list>

namespace fawkes {


/** @class OneToManyDependency <utils/constraints/dependency_onetomany.h>
 * One-to-Many dependency constraint.
 * This dependency constraint models a 1-to-n relationship. There is one
 * object called provider, that any number of other objects (dependants)
 * rely on.
 *
 * The provider is unique and only one provider may exist at any one time.
 * There may be an arbitrary number of dependants. Dependants may only be
 * added if there is already a provider.
 *
 * Dependants can always be removed. The provider can only be removed if
 * there are no more dependants.
 *
 * @author Tim Niemueller
 */

template <class Provider, class Dependant>
  class OneToManyDependency
{
 public:
  OneToManyDependency();
  virtual ~OneToManyDependency();

  virtual void add(Provider *p);
  virtual void add(Dependant *d);
  virtual void remove(Provider *p);
  virtual void remove(Dependant *d);

  virtual bool can_add(Provider *p);
  virtual bool can_add(Dependant *d);
  virtual bool can_remove(Provider *p);
  virtual bool can_remove(Dependant *d);

  virtual Provider *                 provider();
  virtual std::list<Dependant *> &   dependants();

 private:
  Provider                 *_provider;
  std::list<Dependant *>    _dependants;
};


/** Constructor. */
template <class Provider, class Dependant>
  OneToManyDependency<Provider, Dependant>::OneToManyDependency()
{
  _provider = 0;
  _dependants.clear();
}


/** Destructor. */
template <class Provider, class Dependant>
  OneToManyDependency<Provider, Dependant>::~OneToManyDependency()
{
  _dependants.clear();
}


/** Add provider object.
 * This will add the provider to this dependency or throw an exception if there is
 * already a provider.
 * @param p provider object to add
 * @exception DependencyViolationException thrown, if a second provider is added
 */
template <class Provider, class Dependant>
void
OneToManyDependency<Provider, Dependant>::add(Provider *p)
{
  if ( (_provider != 0) && (p != _provider) ) {
    throw DependencyViolationException("Different provider already set");
  } else {
    _provider = p;
  }
}


/** Add dependant object.
 * This will add the dependant to this dependency or throw an exception if there is
 * no provider.
 * @param d dependant object to add
 * @exception DependencyViolationException thrown, if no provider has been set
 */
template <class Provider, class Dependant>
void
OneToManyDependency<Provider, Dependant>::add(Dependant *d)
{
  if (_provider == 0) {
    throw DependencyViolationException("No provider set, cannot accept dependant");
  } else {
    _dependants.push_back(d);
  }
}


/** Remove provider object.
 * @param p provider object to remove
 * @exception DependencyViolationException thrown, if the provider should be removed
 * while there is still at least one dependant.
 */
template <class Provider, class Dependant>
void
OneToManyDependency<Provider, Dependant>::remove(Provider *p)
{
  if ( ! _dependants.empty() ) {
    throw DependencyViolationException("There are still dependants of provider, "
				       "cannot accept removal of provider");
  }
  if ( p == _provider )  _provider = 0;
}


/** Remove a depending object
 * @param d depending object to remove
 */
template <class Provider, class Dependant>
void
OneToManyDependency<Provider, Dependant>::remove(Dependant *d)
{
  if ( d != 0 ) {
    _dependants.remove(d);
  }
}


/** Check if provider can be added.
 * @param p provider object to add
 * @return true, if add(p) would succeed, false otherwise
 */
template <class Provider, class Dependant>
bool
OneToManyDependency<Provider, Dependant>::can_add(Provider *p)
{
  return ( (_provider == 0) || (p == _provider) );
}


/** Check if dependant can be added.
 * @param d dependant object to add
 * @return true, if add(d) would succeed, false otherwise
 */
template <class Provider, class Dependant>
bool
OneToManyDependency<Provider, Dependant>::can_add(Dependant *d)
{
  return (_provider != 0);
}


/** Check if provider can be removed.
 * @param p provider object to remove
 * @return true, if remove(p) would succeed, false otherwise
 */
template <class Provider, class Dependant>
bool
OneToManyDependency<Provider, Dependant>::can_remove(Provider *p)
{
  return _dependants.empty();
}


/** Check if dependant can be removed.
 * @param d depending object to remove
 * @return always true
 */
template <class Provider, class Dependant>
bool
OneToManyDependency<Provider, Dependant>::can_remove(Dependant *d)
{
  return true;
}


/** Get provider.
 * @return provider if set, 0 otherwise
 */
template <class Provider, class Dependant>
Provider *
OneToManyDependency<Provider, Dependant>::provider()
{
  return _provider;
}


/** Get dependants.
 * @return list of dependants.
 */
template <class Provider, class Dependant>
std::list<Dependant *> &
OneToManyDependency<Provider, Dependant>::dependants()
{
  return _dependants;
}


} // end namespace fawkes

#endif
