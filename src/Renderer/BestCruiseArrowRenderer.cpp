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

#include "BestCruiseArrowRenderer.hpp"
#include "Screen/Canvas.hpp"
#include "Screen/Graphics.hpp"
#include "Screen/Fonts.hpp"
#include "Screen/Layout.hpp"
#include "Look/TaskLook.hpp"
#include "Math/Angle.hpp"
#include "Math/Screen.hpp"
#include "NMEA/Derived.hpp"
#include "Units/Units.hpp"
#include "SettingsMap.hpp"
#include "Util/Macros.hpp"

void
BestCruiseArrowRenderer::Draw(Canvas &canvas, const TaskLook &look,
                              const Angle screen_angle,
                              const Angle best_cruise_angle,
                              const RasterPoint pos)
{
  canvas.select(look.best_cruise_track_pen);
  canvas.select(look.best_cruise_track_brush);

  RasterPoint arrow[] = { { -1, -40 }, { -1, -62 }, { -6, -62 }, {  0, -70 },
                          {  6, -62 }, {  1, -62 }, {  1, -40 }, { -1, -40 } };

  PolygonRotateShift(arrow, ARRAY_SIZE(arrow), pos.x, pos.y,
                     best_cruise_angle - screen_angle);
  canvas.polygon(arrow, ARRAY_SIZE(arrow));
}

void
BestCruiseArrowRenderer::Draw(Canvas &canvas, const TaskLook &look,
                              const Angle screen_angle, const RasterPoint pos,
                              const DerivedInfo &calculated)
{
  if (!calculated.task_stats.task_valid ||
      !calculated.task_stats.current_leg.solution_remaining.IsOk() ||
      calculated.task_stats.current_leg.solution_remaining.vector.Distance
      < fixed(0.010))
    return;

  if (calculated.turn_mode == CLIMB)
    return;

  BestCruiseArrowRenderer::Draw(canvas, look, screen_angle,
                                calculated.task_stats.current_leg.
                                solution_remaining.cruise_track_bearing, pos);
}
