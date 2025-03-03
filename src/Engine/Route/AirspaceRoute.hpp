/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2010 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
 */
#ifndef AIRSPACE_ROUTE_HPP
#define AIRSPACE_ROUTE_HPP

#include "RoutePlanner.hpp"
#include "Airspace/Airspaces.hpp"

class AirspaceRoute: public RoutePlanner {
public:
  friend class PrintHelper;

  AirspaceRoute(const Airspaces& master);
  virtual ~AirspaceRoute();

  void synchronise(const Airspaces& master,
                   const AGeoPoint& origin,
                   const AGeoPoint& destination);

  virtual void reset();

  unsigned airspace_size() const;

protected:

  virtual void on_solve(const AGeoPoint& origin,
                        const AGeoPoint& destination);

  virtual bool is_trivial() const {
    return m_airspaces.empty() && RoutePlanner::is_trivial();
  }

private:

  Airspaces m_airspaces;

  typedef std::pair< const AbstractAirspace*, RoutePoint > RouteAirspaceIntersection;

  virtual bool check_clearance(const RouteLink &e, RoutePoint& inp) const;
  virtual void add_nearby(const RouteLink& e);
  virtual bool check_secondary(const RouteLink &e);

  void add_nearby_airspace(const RouteAirspaceIntersection &inx, const RouteLink& e);

  RouteAirspaceIntersection
  first_intersecting(const RouteLink& e) const;

  const AbstractAirspace*
  inside_others(const AGeoPoint& origin) const;

  ClearingPair find_clearing_pair(const SearchPointVector& spv,
                                  const SearchPointVector::const_iterator start,
                                  const SearchPointVector::const_iterator end,
                                  const AFlatGeoPoint &dest) const;

  ClearingPair get_pairs(const SearchPointVector& spv,
                         const RoutePoint& start,
                         const RoutePoint& dest) const;

  ClearingPair get_backup_pairs(const SearchPointVector& spv,
                                const RoutePoint& start,
                                const RoutePoint& intc) const;

  mutable RouteAirspaceIntersection m_inx;
};

#endif
