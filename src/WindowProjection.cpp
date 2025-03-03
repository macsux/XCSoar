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

#include "WindowProjection.hpp"

bool
WindowProjection::GeoVisible(const GeoPoint &loc) const
{
  return screenbounds_latlon.inside(loc);
}

bool
WindowProjection::GeoToScreenIfVisible(const GeoPoint &loc, RasterPoint &sc) const
{
  if (GeoVisible(loc)) {
    sc = GeoToScreen(loc);
    return ScreenVisible(sc);
  }

  return false;
}

bool
WindowProjection::ScreenVisible(const RasterPoint &P) const
{
  assert(screen_size_initialised);

  return P.x >= 0 && (unsigned)P.x < screen_width &&
    P.y >= 0 && (unsigned)P.y < screen_height;
}

fixed
WindowProjection::GetScreenDistanceMeters() const
{
  return DistancePixelsToMeters(GetScreenDistance());
}

GeoPoint
WindowProjection::GetGeoScreenCenter() const
{
  return ScreenToGeo(GetScreenWidth() / 2, GetScreenHeight() / 2);
}

void
WindowProjection::UpdateScreenBounds()
{
  assert(screen_size_initialised);

  GeoBounds sb(ScreenToGeo(0, 0));
  sb.extend(ScreenToGeo(screen_width, 0));
  sb.extend(ScreenToGeo(screen_width, screen_height));
  sb.extend(ScreenToGeo(0, screen_height));

  screenbounds_latlon = sb;
}
