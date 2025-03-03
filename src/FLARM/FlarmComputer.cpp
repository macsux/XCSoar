/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2011 The XCSoar Project
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

#include "FLARM/FlarmComputer.hpp"
#include "FLARM/FlarmDetails.hpp"
#include "NMEA/Info.hpp"
#include "Engine/Math/Earth.hpp"
#include "Engine/Navigation/Geometry/GeoVector.hpp"

void
FlarmComputer::Process(FLARM_STATE &flarm, const FLARM_STATE &last_flarm,
                       const NMEAInfo &basic)
{
  // if (FLARM data is available)
  if (!flarm.available || flarm.traffic.empty())
    return;

  fixed FLARM_NorthingToLatitude(0);
  fixed FLARM_EastingToLongitude(0);

  if (basic.location_available) {
    // Precalculate relative east and north projection to lat/lon
    // for Location calculations of each target
    Angle delta_lat = Angle::degrees(fixed(0.01));
    Angle delta_lon = Angle::degrees(fixed(0.01));

    GeoPoint plat = basic.location;
    plat.Latitude += delta_lat;
    GeoPoint plon = basic.location;
    plon.Longitude += delta_lon;

    fixed dlat = Distance(basic.location, plat);
    fixed dlon = Distance(basic.location, plon);

    if (positive(fabs(dlat)) && positive(fabs(dlon))) {
      FLARM_NorthingToLatitude = delta_lat.value_degrees() / dlat;
      FLARM_EastingToLongitude = delta_lon.value_degrees() / dlon;
    }
  }

  // for each item in traffic
  for (unsigned i = 0; i < flarm.traffic.size(); i++) {
    FLARM_TRAFFIC &traffic = flarm.traffic[i];

    // if we don't know the target's name yet
    if (!traffic.HasName()) {
      // lookup the name of this target's id
      const TCHAR *fname = FlarmDetails::LookupCallsign(traffic.id);
      if (fname != NULL)
        traffic.name = fname;
    }

    // Calculate distance
    traffic.distance = hypot(traffic.relative_north, traffic.relative_east);

    // Calculate Location
    traffic.location_available = basic.location_available;
    if (traffic.location_available) {
      traffic.location.Latitude =
          Angle::degrees(traffic.relative_north * FLARM_NorthingToLatitude) +
          basic.location.Latitude;

      traffic.location.Longitude =
          Angle::degrees(traffic.relative_east * FLARM_EastingToLongitude) +
          basic.location.Longitude;
    }

    // Calculate absolute altitude
    traffic.altitude_available = basic.gps_altitude_available;
    if (traffic.altitude_available)
      traffic.altitude = traffic.relative_altitude + basic.gps_altitude;

    // Calculate average climb rate
    traffic.climb_rate_avg30s_available = traffic.altitude_available;
    if (traffic.climb_rate_avg30s_available)
      traffic.climb_rate_avg30s =
        flarmCalculations.Average30s(traffic.id, basic.time, traffic.altitude);

    // The following calculations are only relevant for targets
    // where information is missing
    if (traffic.track_received || traffic.turn_rate_received ||
        traffic.speed_received || traffic.climb_rate_received)
      continue;

    // Check if the target has been seen before in the last seconds
    const FLARM_TRAFFIC *last_traffic = last_flarm.FindTraffic(traffic.id);
    if (last_traffic == NULL || !last_traffic->valid)
      continue;

    // Calculate the time difference between now and the last contact
    fixed dt = traffic.valid.GetTimeDifference(last_traffic->valid);
    if (positive(dt)) {
      // Calculate the immediate climb rate
      if (!traffic.climb_rate_received)
        traffic.climb_rate =
          (traffic.relative_altitude - last_traffic->relative_altitude) / dt;
    } else {
      // Since the time difference is zero (or negative)
      // we can just copy the old values
      if (!traffic.climb_rate_received)
        traffic.climb_rate = last_traffic->climb_rate;
    }

    if (positive(dt) &&
        traffic.location_available &&
        last_traffic->location_available) {
      // Calculate the GeoVector between now and the last contact
      GeoVector vec = last_traffic->location.distance_bearing(traffic.location);

      if (!traffic.track_received)
        traffic.track = vec.Bearing;

      // Calculate the turn rate
      if (!traffic.turn_rate_received) {
        Angle turn_rate = traffic.track - last_traffic->track;
        traffic.turn_rate =
          turn_rate.as_delta().value_degrees() / dt;
      }

      // Calculate the speed [m/s]
      if (!traffic.speed_received)
        traffic.speed = vec.Distance / dt;
    } else {
      // Since the time difference is zero (or negative)
      // we can just copy the old values
      if (!traffic.track_received)
        traffic.track = last_traffic->track;

      if (!traffic.turn_rate_received)
        traffic.turn_rate = last_traffic->turn_rate;

      if (!traffic.speed_received)
        traffic.speed = last_traffic->speed;
    }
  }
}
