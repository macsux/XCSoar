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

#include "Device/Driver/Flytec.hpp"
#include "Device/Parser.hpp"
#include "Device/Driver.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "Units/Units.hpp"
#include <string.h>
#include <stdlib.h>
#include <math.h>

class FlytecDevice : public AbstractDevice {
private:
  Port *port;

public:
  FlytecDevice(Port *_port):port(_port) {}

public:
  virtual bool ParseNMEA(const char *line, struct NMEAInfo &info);
};

/**
 * Parse a "$BRSF" sentence.
 *
 * Example: "$BRSF,063,-013,-0035,1,193,00351,535,485*38"
 */
static bool
FlytecParseBRSF(NMEAInputLine &line, NMEAInfo &info)
{
  fixed value;

  // 0 = indicated or true airspeed [km/h]
  if (line.read_checked(value))
    // XXX is that TAS or IAS?  Documentation isn't clear.
    info.ProvideBothAirspeeds(Units::ToSysUnit(value, unKiloMeterPerHour));

  // 1 = integrated vario [dm/s]
  // 2 = altitude A2 [m] (XXX what's this?)
  // 3 = waypoint
  // 4 = bearing to waypoint [degrees]
  // 5 = distance to waypoint [100m]
  // 6 = MacCready speed to fly [100m/h]
  // 7 = speed to fly, best glide [100m/h]

  return true;
}

/**
 * Parse a "$VMVABD" sentence.
 *
 * Example: "$VMVABD,0000.0,M,0547.0,M,-0.0,,,MS,0.0,KH,22.4,C*65"
 */
static bool
FlytecParseVMVABD(NMEAInputLine &line, NMEAInfo &info)
{
  fixed value;

  // 0,1 = GPS altitude, unit
  if (line.read_checked_compare(info.gps_altitude, "M"))
    info.gps_altitude_available.Update(info.clock);

  // 2,3 = baro altitude, unit
  if (line.read_checked_compare(value, "M"))
    info.ProvideBaroAltitudeTrue(value);

  // 4-7 = integrated vario, unit
  line.skip(4);

  // 8,9 = indicated or true airspeed, unit
  if (line.read_checked_compare(value, "KH"))
    // XXX is that TAS or IAS?  Documentation isn't clear.
    info.ProvideBothAirspeeds(Units::ToSysUnit(value, unKiloMeterPerHour));

  // 10,11 = temperature, unit
  info.temperature_available =
    line.read_checked_compare(value, "C");
  if (info.temperature_available)
    info.temperature = Units::ToSysUnit(value, unGradCelcius);

  return true;
}

/**
 * Parse a "$FLYSEN" sentence.
 *
 * @see http://www.flytec.ch/public/Special%20NMEA%20sentence.pdf
$FLYSEN                                   8 Digits
Date(ddmmyy)                              6 Digits
Time(hhmmss)                              6 Digits
Latitude(ddmm.mmm)                        8 Digits incl. decimal
N (or S)                                  1 Digit
Longitude(dddmm.mmm)                      9 Digits inc. decimal
E (or W)                                  1 Digit
Track (xxx Deg)                           3 Digits (000 to 359)
Speed over Ground (xxxxx cm/s)            5 Digits
GPS altitude (xxxxx meter)                5 Digits
Validity of 3 D fix A or V                1 Digit In case of V (void=not valid) GPS data should not be used. GPS altitude position and speed should be ignored.
Satellites in Use (0 to 12)               2 Digits
Raw pressure (xxxxxx Pa)                  6 Digits
Baro Altitude (xxxxx meter)               5 Digits (-xxxx to xxxxx)(Based on 1013.25hPa)
Variometer (xxxx cm/s)                    4 or 5 Digits (-9999 to 9999)
true airspeed (xxxxx cm/s)                5 Digits (0 to 99999cm/s = 3600km/h)
Airspeed source P or V                    1 Digit P= pitot			 V = Vane wheel
Temp. PCB (xxx °C)                        3 Digits (-99 to 999)
Temp. Balloon Envelope (xxx °C)           3 Digits (-99 to 999)
Battery Capacity Bank 1 (0 to 100%)       3 Digits 999 if data is not available (no battery or other reason)
Battery Capacity Bank 2 (0 to 100%)       3 Digits 999 if data is not available (no battery or other reason)
Speed to fly1 (MC0 xxxxx cm/s)            5 Digits
Speed to fly2 (McC. xxxxx cm/s)			      5 Digits
Keypress Code (Experimental empty to 99)  3 Digits (Code see below)
Dist. to WP (xxxxxx m)				 	          6 Digits (Max 200000m) 999999 if not available (GPS position not available or other reason)
Bearing (xxx Deg)				 	                3 Digits (999 if not available
*Checksum<CR><LF> 5 Digits
Total 115 Digits
 */
static bool
FlytecParseFLYSEN(NMEAInputLine &line, NMEAInfo &info)
{
  fixed value;


  //Date(ddmmyy), 6 Digits
  line.skip();

  //  Time(hhmmss),   6 Digits
  line.skip();

  //  Latitude(ddmm.mmm),   8 Digits incl. decimal
  char position[10];
  char direction = '\0';
  char degreesArr[10];
  char minsArr[10];

  for (int i=0;i<10;i++)
    position[i] = '\0';
  for (int i=0;i<10;i++)
    degreesArr[i] = '\0';
  for (int i=0;i<10;i++)
    minsArr[i] = '\0';

  Angle lat;
  Angle lon;
  double degreesLat = 0.0;
  double minsLat = 0.0;
  line.read(position,8);
  for(int i=0;i<2;i++)
	  degreesArr[i] = position[i];
  for(int i=0;i<5;i++)
  	  minsArr[i] = position[i+2];
  degreesLat = fabs(atof(degreesArr));
  minsLat = fabs(atof(minsArr));


  //N (or S), 1 Digit

  direction = line.read_first_char();
  if(direction == 'S')
    lat = Angle::dms(-fixed(degreesLat),-fixed(minsLat),fixed(0));
  else
    lat = Angle::dms(fixed(degreesLat),fixed(minsLat),fixed(0));


  //Longitude(dddmm.mmm), 9 Digits inc. decimal
  for (int i=0;i<10;i++)
    position[i] = '\0';
  for (int i=0;i<10;i++)
    degreesArr[i] = '\0';
  for (int i=0;i<10;i++)
    minsArr[i] = '\0';
  double degreesLon = 0.0;
  double minsLon = 0.0;
  line.read(position,9);
	for(int i=0;i<3;i++)
	  degreesArr[i] = position[i];
	for(int i=0;i<6;i++)
		  minsArr[i] = position[i+3];
	degreesLon = fabs(atof(degreesArr));
	minsLon = fabs(atof(minsArr));

	//E (or W), 1 Digit
	direction = line.read_first_char();
  if(direction == 'W')
    lon = Angle::dms(-fixed(degreesLon),-fixed(minsLon),fixed(0));
  else
    lon = Angle::dms(fixed(degreesLon),fixed(minsLon),fixed(0));

  //Track (xxx Deg)              3 Digits (000 to 359)
  if(line.read_checked(value))
  {
    info.track = Angle::degrees(fixed(value));
    info.track_available.Update(info.clock);
  }

  //Speed over Ground (xxxxx cm/s)       5 Digits
  double groundSpeed = 0;
    groundSpeed = line.read(groundSpeed) / 100;



  //GPS altitude (xxxxx meter)         5 Digits
  double gpsAltitude = 0;
    gpsAltitude = line.read(gpsAltitude);


  // Validity of 3 D fix A or V         1 Digit In case of V (void=not valid) GPS data should not be used. GPS altitude position and speed should be ignored.
  char validity = line.read_first_char();
  if(validity != 'V')
  {
    // gps data is good - update the info
    info.location.Latitude = lat;
    info.location.Longitude = lon;
    info.location_available.Update(info.clock);
    info.ground_speed = groundSpeed;
    info.ground_speed_available.Update(info.clock);
    info.gps_altitude = gpsAltitude;
    info.gps_altitude_available.Update(info.clock);

  }

  //Satellites in Use (0 to 12)        2 Digits
  line.read(info.gps.satellites_used);

  //Raw pressure (xxxxxx Pa)         6 Digits
  if(line.read_checked(value))
    info.ProvideStaticPressure(value / 100);

  //Baro Altitude (xxxxx meter)        5 Digits (-xxxx to xxxxx)(Based on 1013.25hPa)
  if(line.read_checked(value))
    info.ProvideBaroAltitudeTrue(value);


  //Variometer (xxxx cm/s)                    4 or 5 Digits (-9999 to 9999)
  if(line.read_checked(value))
    info.ProvideTotalEnergyVario(value / 100);

  //true airspeed (xxxxx cm/s)                5 Digits (0 to 99999cm/s = 3600km/h)
  if(line.read_checked(value))
    info.ProvideTrueAirspeed(value / 100);

  //Airspeed source P or V                    1 Digit P= pitot       V = Vane wheel
  line.skip();

  //Temp. PCB (xxx °C)                        3 Digits (-99 to 999)
  if(line.read_checked(value))
  {
    info.temperature = value;
    info.temperature_available = true;
  }

  //Temp. Balloon Envelope (xxx °C)           3 Digits (-99 to 999)
  line.skip();

  //Battery Capacity Bank 1 (0 to 100%)       3 Digits 999 if data is not available (no battery or other reason)
  line.skip();

  //Battery Capacity Bank 2 (0 to 100%)       3 Digits 999 if data is not available (no battery or other reason)
  line.skip();


  //Speed to fly1 (MC0 xxxxx cm/s)            5 Digits
  if(line.read_checked(value))
    info.settings.ProvideMacCready(value / 100, info.clock);

  //Speed to fly2 (McC. xxxxx cm/s)           5 Digits
  line.skip();

  //Keypress Code (Experimental empty to 99)  3 Digits (Code see below)
  line.skip();

  //Dist. to WP (xxxxxx m)                    6 Digits (Max 200000m) 999999 if not available (GPS position not available or other reason)
  line.skip();

  //Bearing (xxx Deg)                         3 Digits (999 if not available
  line.skip();


  return true;
}

bool
FlytecDevice::ParseNMEA(const char *_line, NMEAInfo &info)
{
  NMEAInputLine line(_line);
  char type[16];
  line.read(type, 16);

  if (strcmp(type, "$BRSF") == 0)
    return FlytecParseBRSF(line, info);
  else if (strcmp(type, "$VMVABD") == 0)
    return FlytecParseVMVABD(line, info);
  else if (strcmp(type, "$FLYSEN") == 0)
    return FlytecParseFLYSEN(line, info);
  else
    return false;
}

static Device *
FlytecCreateOnPort(const DeviceConfig &config, Port *com_port)
{
  return new FlytecDevice(com_port);
}



const struct DeviceRegister flytec_device_driver = {
  _T("Flytec"),
  _T("Flytec 5030 / Brauniger"),
  0,
  FlytecCreateOnPort,
};
