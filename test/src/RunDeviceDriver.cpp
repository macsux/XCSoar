/*
Copyright_License {

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

#include "NMEA/Info.hpp"
#include "Device/NullPort.hpp"
#include "Device/Driver.hpp"
#include "Device/Register.hpp"
#include "Device/Parser.hpp"
#include "Device/device.hpp"
#include "Engine/Navigation/GeoPoint.hpp"
#include "Engine/Waypoint/Waypoints.hpp"
#include "InputEvents.hpp"
#include "OS/PathName.hpp"
#include "Profile/DeviceConfig.hpp"

#include <stdio.h>

const struct DeviceRegister *driver;

Waypoints way_points;
Waypoints::Waypoints() {}

/*
 * Fake InputEvents.cpp
 */

bool
InputEvents::processNmea(unsigned key)
{
  return true;
}

/*
 * The actual code.
 */

static void
Dump(GeoPoint location)
{
  int latitude = (int)(location.Latitude.value_degrees() * 3600);
  char north_or_south = latitude < 0 ? 'S' : 'N';
  latitude = abs(latitude);

  int longitude = (int)(location.Longitude.value_degrees() * 3600);
  char east_or_west = latitude < 0 ? 'W' : 'E';
  longitude = abs(longitude);

  printf("%d.%02u.%02u%c;%d.%02u.%02u%c",
         latitude / 3600, (latitude / 60) % 60, latitude % 60, north_or_south,
         longitude / 3600, (longitude / 60) % 60, longitude % 60, east_or_west);
}

static void
Dump(const ExternalSettings &settings)
{
  if (settings.mac_cready_available)
    printf("MacCready=%.1f\n", (double)settings.mac_cready);

  if (settings.ballast_fraction_available)
    printf("Ballast=%.1f\n", (double)settings.ballast_fraction);

  if (settings.bugs_available)
    printf("Bugs=%.1f\n", (double)settings.bugs);

  if (settings.qnh_available)
    printf("QNH=%f\n", (double)settings.qnh.GetQNH());
}

static void
Dump(const NMEAInfo &basic)
{
  printf("Date=%02u.%02u.%04u\n",
         basic.date_time_utc.day, basic.date_time_utc.month, basic.date_time_utc.year);
  printf("Time=%02u:%02u:%02u\n",
         basic.date_time_utc.hour, basic.date_time_utc.minute, basic.date_time_utc.second);

  if (!basic.connected)
    printf("GPS not connected\n");
  else
    printf("GPS connected, %d satellites\n", basic.gps.satellites_used);

  if (basic.location_available) {
    printf("Position=");
    Dump(basic.location);
    printf("\n");
  }

  if (basic.track_available)
    printf("TrackBearing=%d\n", (int)basic.track.value_degrees());

  if (basic.ground_speed_available)
    printf("GroundSpeed=%d\n", (int)basic.ground_speed);

  if (basic.airspeed_available) {
    printf("TrueAirspeed=%d\n", (int)basic.true_airspeed);
    printf("IndicatedAirspeed=%d\n",
           (int)basic.indicated_airspeed);
  }

  if (basic.gps_altitude_available)
    printf("GPSAltitude=%d\n", (int)basic.gps_altitude);

  if (basic.static_pressure_available)
    printf("StaticPressure=%d\n", (int)basic.static_pressure);

  if (basic.pressure_altitude_available)
    printf("PressureAltitude=%d\n", (int)basic.pressure_altitude);

  if (basic.baro_altitude_available)
    printf("BaroAltitude=%d\n", (int)basic.baro_altitude);

  if (basic.total_energy_vario_available)
    printf("TotalEnergyVario=%.1f\n", (double)basic.total_energy_vario);

  if (basic.netto_vario_available)
    printf("NettoVario=%.1f\n", (double)basic.netto_vario);

  if (basic.external_wind_available)
    printf("Wind=%d/%d\n",
           (int)basic.external_wind.bearing.value_degrees(),
           (int)basic.external_wind.norm);

  if (basic.temperature_available)
    printf("OutsideAirTemperature=%d\n", (int)basic.temperature);

  if (basic.humidity_available)
    printf("RelativeHumidity=%d\n", (int)basic.humidity);

  const FLARM_STATE &flarm = basic.flarm;
  if (flarm.available) {
    printf("FLARM rx=%u tx=%u\n", flarm.rx, flarm.tx);
    printf("FLARM gps=%u\n", (unsigned)flarm.gps);
    printf("FLARM alarm=%u\n", (unsigned)flarm.alarm_level);
    printf("FLARM traffic=%u new=%d\n",
           flarm.traffic.size(), flarm.NewTraffic);
  }

  if (basic.engine_noise_level_available)
    printf("ENL=%u\n", basic.engine_noise_level);

  Dump(basic.settings);
}

int main(int argc, char **argv)
{
  if (argc != 2) {
    fprintf(stderr, "Usage: %s DRIVER\n"
            "Where DRIVER is one of:\n", argv[0]);

    const TCHAR *name;
    for (unsigned i = 0; (name = GetDriverNameByIndex(i)) != NULL; ++i)
      _ftprintf(stderr, _T("\t%s\n"), name);

    return 1;
  }

  PathName driver_name(argv[1]);
  driver = FindDriverByName(driver_name);
  if (driver == NULL) {
    fprintf(stderr, "No such driver: %s\n", argv[1]);
    return 1;
  }

  DeviceConfig config;
  config.Clear();

  NullPort port;
  Device *device = driver->CreateOnPort != NULL
    ? driver->CreateOnPort(config, &port)
    : NULL;

  NMEAParser parser;

  NMEAInfo data;
  data.Reset();

  char buffer[1024];
  while (fgets(buffer, sizeof(buffer), stdin) != NULL) {
    TrimRight(buffer);

    if (device == NULL || !device->ParseNMEA(buffer, data))
      parser.ParseNMEAString_Internal(buffer, data);
  }

  Dump(data);

  return EXIT_SUCCESS;
}
