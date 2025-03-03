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

#include "Dialogs/Waypoint.hpp"
#include "Dialogs/Internal.hpp"
#include "Dialogs/TextEntry.hpp"
#include "Math/Earth.hpp"
#include "Screen/Layout.hpp"
#include "Screen/Fonts.hpp"
#include "Compatibility/string.h"
#include "Math/FastMath.h"
#include "DataField/Base.hpp"
#include "Profile/Profile.hpp"
#include "OS/PathName.hpp"
#include "Waypoint/Waypoints.hpp"
#include "Waypoint/WaypointVisitor.hpp"
#include "Components.hpp"
#include "Compiler.h"
#include "DataField/Enum.hpp"
#include "LogFile.hpp"
#include "StringUtil.hpp"
#include "Task/Tasks/OrderedTask.hpp"
#include "Task/Factory/AbstractTaskFactory.hpp"
#include "MainWindow.hpp"
#include "Look/Look.hpp"
#include "Util/Macros.hpp"
#include "Renderer/WaypointListRenderer.hpp"

#include <algorithm>
#include <list>

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

class FAITrianglePointValidator;

static GeoPoint g_location;
static FAITrianglePointValidator *FAITriPtVali = NULL;
static WndForm *wf = NULL;
static WndListFrame *wWaypointList = NULL;
static WndButton *wbName;
static WndProperty *wpDistance;
static WndProperty *wpDirection;
static WndProperty *wpType;

static const fixed DistanceFilter[] = {
  fixed_zero, fixed(25.0), fixed(50.0),
  fixed(75.0), fixed(100.0), fixed(150.0),
  fixed(250.0), fixed(500.0), fixed(1000.0),
};

#define DirHDG -1
static int DirectionFilter[] = {
  0, DirHDG, 360, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330
};

static Angle last_heading = Angle::zero();

/**
 * used for single-letter name search with Left/Right keys
 */
static int NameFilterIdx = -1;

static const TCHAR *const TypeFilter[] = {
  _T("*"), _T("Airport"), _T("Landable"),
  _T("Turnpoint"), 
  _T("Start"), 
  _T("Finish"), 
  _T("Left FAI Triangle"),
  _T("Right FAI Triangle"),
  _T("File 1"), _T("File 2"),
  _T("Recently Used"),
  NULL
};

enum type_filter {
  tfAll = 0,
  tfAirport,
  tfLandable,
  tfTurnpoint,
  tfStart,
  tfFinish,
  tfFAITriangleLeft,
  tfFAITriangleRight,
  tfFile1,
  tfFile2,
  tfLastUsed,
};

enum {
  NAMEFILTERLEN = 10,
};

struct WaypointFilterData
{
  TCHAR name[NAMEFILTERLEN + 1];

  int distance_index;
  int direction_index;
  type_filter type_index;

  bool defined() const {
    return !string_is_empty(name) || distance_index > 0 ||
      direction_index > 0 || type_index > 0;
  }
};

static WaypointFilterData filter_data;

/**
 * Structure to hold Waypoint sorting information
 */
struct WaypointSelectInfo
{
  /** Pointer to actual waypoint (unprotected!) */
  const Waypoint* way_point;
  /** Distance in user units from observer to waypoint */
  fixed Distance;
  /** Bearing (deg true north) from observer to waypoint */
  Angle Direction;
};

struct WaypointSelectInfoVector :
  public std::vector<WaypointSelectInfo>
{
  void push_back(const Waypoint &way_point, const GeoPoint &Location) {
    WaypointSelectInfo info;

    info.way_point = &way_point;

    const GeoVector vec(Location, way_point.location);

    info.Distance = vec.Distance;
    info.Direction = vec.Bearing;

    std::vector<WaypointSelectInfo>::push_back(info);
  }
};

static WaypointSelectInfoVector WaypointSelectInfo;
static std::list<unsigned int> LastUsedWaypointNames;

static TCHAR *
GetDirectionData(int DirectionFilterIdx)
{
  static TCHAR sTmp[12];

  if (DirectionFilterIdx == 0)
    _stprintf(sTmp, _T("%c"), '*');
  else if (DirectionFilterIdx == 1)
    _stprintf(sTmp, _T("HDG(%u")_T(DEG)_T(")"),
              uround(last_heading.as_bearing().value_degrees()));
  else
    _stprintf(sTmp, _T("%d")_T(DEG), DirectionFilter[DirectionFilterIdx]);

  return sTmp;
}

static void
InitializeDirection(bool bOnlyHeading)
{
  // initialize datafieldenum for Direction
  if (wpDirection) {
    DataFieldEnum* dfe;
    dfe = (DataFieldEnum*)wpDirection->GetDataField();
    if (!bOnlyHeading) {
      for (unsigned int i = 0; i < ARRAY_SIZE(DirectionFilter); i++)
        dfe->addEnumText(GetDirectionData(i));

      dfe->SetAsInteger(filter_data.direction_index);
    }
    // update heading value to current heading
    dfe->replaceEnumText(1,GetDirectionData(1));
    wpDirection->RefreshDisplay();
  }
}

static void
PrepareData(void)
{
  TCHAR sTmp[15];

  filter_data.name[0] = _T('\0');

  wbName->SetCaption(_T("*"));

  // initialize datafieldenum for Distance
  if (wpDistance) {
    DataFieldEnum* dfe;
    dfe = (DataFieldEnum*)wpDistance->GetDataField();
    dfe->addEnumText(_T("*"));
    for (unsigned i = 1; i < ARRAY_SIZE(DistanceFilter); i++) {
      _stprintf(sTmp, _T("%.0f%s"), (double)DistanceFilter[i],
                Units::GetDistanceName());
      dfe->addEnumText(sTmp);
    }
    dfe->SetAsInteger(filter_data.distance_index);
    wpDistance->RefreshDisplay();
  }

  InitializeDirection(false);

  // initialize datafieldenum for Type
  if (wpType) {
    DataFieldEnum* dfe;
    dfe = (DataFieldEnum*)wpType->GetDataField();
    dfe->addEnumTexts(TypeFilter);

    TCHAR szFile[MAX_PATH];
    const TCHAR * t;
    if (Profile::GetPath(szProfileWaypointFile, szFile)) {
      t = BaseName(szFile);
      dfe->replaceEnumText(tfFile1, t);
    }
    if (Profile::GetPath(szProfileAdditionalWaypointFile, szFile)) {
      t = BaseName(szFile);
      dfe->replaceEnumText(tfFile2, t);
    }
    dfe->SetAsInteger(filter_data.type_index);
    wpType->RefreshDisplay();
  }
}

/**
 * Helper class to filter waypoints in list based on whether the filter is set
 * to Right or Left FAI Triangle.
 */
class FAITrianglePointValidator
{
public:
  FAITrianglePointValidator(OrderedTask *ordered_task,
                            const unsigned ordered_task_index) :
                            task(ordered_task),
                            t_index(ordered_task_index),
                            t_size(0),
                            leg1(fixed_zero),
                            leg2(fixed_zero),
                            leg3(fixed_zero),
                            minFAILeg(fixed(2000)),
                            minFAIAngle(fixed(31.5)),
                            maxFAIAngle(fixed(114)),
                            FAITrianglePointInvalid(false)
  {
    PrepareFAITest(ordered_task, ordered_task_index);
  }


  bool
  TestFAITriangle(const fixed d1, const fixed d2, const fixed d3)
  {
    if ((d1 < minFAILeg) || (d2 < minFAILeg) || (d3 < minFAILeg))
      return false;

    return AbstractTaskFactory::TestFAITriangle(d1, d2, d3);
  }

  /**
   * Perform fast check to exclude point as from further consideration
   * based on min/max possible values for any FAI triangle
   * @param p0 point 1 of angle
   * @param p1 point 2 of angle
   * @param p2 point 3 of angle
   * @param right.  = 1 if angle is for right triangle, -1 if left triangle.
   * @returns False if angle from three points is out of possible range for
   * an FAI triangle.
   */
  bool
  isFAIAngle(const GeoPoint &p0, const GeoPoint &p1, const GeoPoint &p2,
             const fixed right)
  {
    const Angle a01 = p0.bearing(p1);
    const Angle a21 = p2.bearing(p1);
    const fixed diff = (a01 - a21).as_delta().value_degrees();

    if (positive(right))
      return (diff > minFAIAngle) && (diff < maxFAIAngle);
    else
      return (diff < fixed(-1) * minFAIAngle) && (diff > fixed(-1)
              * maxFAIAngle);
  }

  /** Test whether wp could be a point in an FAI triangle based on the other
   * points in the task and the current ordered task index
   * Tests angle ranges first to reduce more costly calculation of distances
   * @param wp Point being tested
   * @param right = 1 if triangle turns are to right, -1 if turns are to left
   * @return True if point would be valid in an FAI Triangle
   */
  bool
  isFAITrianglePoint(const Waypoint& wp, const fixed right)
  {
    if (FAITrianglePointInvalid)
      return false;

    if (!task)
      return true;

    if (t_size == 0)
      return true;

    const GeoPoint &p = wp.location;
    // replace start
    if (t_index == 0) {
      assert(t_size <= 4 && t_size > 0);

      switch (t_size) {
      case 1:
        return true;

      case 2:
        return p.distance(task->get_tp(1)->GetLocation()) > minFAILeg;

      default: // size == 3 or 4
        if (!isFAIAngle(p, task->get_tp(1)->GetLocation(),
                        task->get_tp(2)->GetLocation(), right))
          return false;
        if (t_size == 3) {
          return TestFAITriangle(p.distance(task->get_tp(1)->GetLocation()),
                                 leg2,
                                 task->get_tp(2)->GetLocation().distance(p));
        } else if (t_size == 4) {
          return (wp == task->get_tp(3)->GetWaypoint()) &&
                 TestFAITriangle(p.distance(task->get_tp(1)->GetLocation()),
                                 leg2,
                                 leg3);
        }
      }
    }
    // append or replace point #1
    if (t_index == 1) {
      assert(t_size > 0);

      if (t_size <= 2)
        return p.distance(task->get_tp(0)->GetLocation()) > minFAILeg;

      // size == 3 or 4
      if (!isFAIAngle(task->get_tp(0)->GetLocation(),
                      p,
                      task->get_tp(2)->GetLocation(), right))
        return false;

      if (t_size == 3) {
        return TestFAITriangle(p.distance(task->get_tp(0)->GetLocation()),
                               p.distance(task->get_tp(2)->GetLocation()),
                               task->get_tp(2)->GetLocation().
                                  distance(task->get_tp(0)->GetLocation()));
      } else if (t_size == 4) {
        return TestFAITriangle(p.distance(task->get_tp(0)->GetLocation()),
                               p.distance(task->get_tp(2)->GetLocation()),
                               leg3);
      }
    }
    // append or replace point #2
    if (t_index == 2) {
      assert(t_size >= 2);
      if (!isFAIAngle(task->get_tp(0)->GetLocation(),
                      task->get_tp(1)->GetLocation(),
                      p, right))
        return false;

      if (t_size < 4) { // no finish point yet
        return TestFAITriangle(leg1,
                               p.distance(task->get_tp(1)->GetLocation()),
                               p.distance(task->get_tp(0)->GetLocation()));

      } else { // already finish point(#3) exists
        return (task->get_tp(0)->GetWaypoint() == task->get_tp(3)->GetWaypoint()) &&
                TestFAITriangle(leg1,
                                p.distance(task->get_tp(1)->GetLocation()),
                                p.distance(task->get_tp(0)->GetLocation()));
      }
    }
    // append or replace finish
    if (t_index == 3) {
      assert (t_size == 3 || t_size == 4);
      return (wp == task->get_tp(0)->GetWaypoint()) &&
              TestFAITriangle(leg1,
                              leg2,
                              p.distance(task->get_tp(2)->GetLocation()));
    }
    return true;
  }

private:

  void
  PrepareFAITest(OrderedTask *ordered_task, const unsigned ordered_task_index)
  {
    task = ordered_task;
    t_index = ordered_task_index;

    FAITrianglePointInvalid = false;

    if (ordered_task) {
      t_size = task->TaskSize();
      leg1 = (t_size > 1) ? task->getTaskPoint(1)->
              GetVectorPlanned().Distance : fixed_zero;
      leg2 = (t_size > 2) ? task->getTaskPoint(2)->
              GetVectorPlanned().Distance : fixed_zero;
      leg3 = (t_size > 3) ? task->getTaskPoint(3)->
              GetVectorPlanned().Distance : fixed_zero;
    } else {
      leg1 = leg2 = leg3 = fixed_zero;
      t_size = 0;
      t_index = 0;
    }

    if (t_size > 4)
      FAITrianglePointInvalid = true;

    if (t_index > 3)
      FAITrianglePointInvalid = true;
  }

private:
  OrderedTask *task;
  unsigned t_index;
  unsigned t_size;
  fixed leg1;
  fixed leg2;
  fixed leg3;

  /** min distance for any FAI Leg -- derived from circular FAI sector radius */
  const fixed minFAILeg;
  /** min angle allowable in a FAI Triangle 31.5 degrees */
  const fixed minFAIAngle;
  /** max angle allowable in a FAI Triangle 113.2 degrees */
  const fixed maxFAIAngle;
  bool FAITrianglePointInvalid;
};

class FilterWaypointVisitor:
  public WaypointVisitor,
  private WaypointFilterData
{
  const GeoPoint location;
  const Angle heading;
  WaypointSelectInfoVector &vector;

private:
  static bool
  compare_type(const Waypoint &wp, type_filter type_index)
  {
    switch (type_index) {
    case tfAll:
      return true;

    case tfAirport:
      return wp.IsAirport();

    case tfLandable:
      return wp.IsLandable();

    case tfTurnpoint:
      return wp.IsTurnpoint();

    case tfStart:
      return wp.IsStartpoint();

    case tfFinish:
      return wp.IsFinishpoint();

    case tfFAITriangleLeft:
      return FAITriPtVali->isFAITrianglePoint(wp, fixed(-1));

    case tfFAITriangleRight:
      return FAITriPtVali->isFAITrianglePoint(wp, fixed_one);

    case tfFile1:
      return wp.file_num == 1;

    case tfFile2:
      return wp.file_num == 2;

    case tfLastUsed:
      return false;
    }

    /* not reachable */
    return false;
  }

  static bool
  compare_direction(const Waypoint &wp, int direction_index,
                    GeoPoint location, Angle heading)
  {
    if (direction_index <= 0)
      return true;

    int a = DirectionFilter[filter_data.direction_index];
    Angle angle = (a == DirHDG) ? heading : Angle::degrees(fixed(a));

    const GeoVector vec(location, wp.location);
    fixed DirectionErr = (vec.Bearing - angle).as_delta().magnitude_degrees();

    return DirectionErr < fixed(18);
  }

  static bool
  compare_name(const Waypoint &wp, const TCHAR *name)
  {
    return _tcsnicmp(wp.name.c_str(), name, _tcslen(name)) == 0;
  }

public:
  FilterWaypointVisitor(const WaypointFilterData &filter,
                        GeoPoint _location, Angle _heading,
                        WaypointSelectInfoVector &_vector)
    :WaypointFilterData(filter), location(_location), heading(_heading),
     vector(_vector) {}

  void Visit(const Waypoint &wp) {
    if (compare_type(wp, type_index) &&
        (filter_data.distance_index == 0 || compare_name(wp, name)) &&
        compare_direction(wp, direction_index, location, heading))
      vector.push_back(wp, location);
  }
};

static bool
WaypointDistanceCompare(const struct WaypointSelectInfo &a,
                        const struct WaypointSelectInfo &b)
{
  return a.Distance < b.Distance;
}

static void
FillList(WaypointSelectInfoVector &dest, const Waypoints &src,
         GeoPoint location, Angle heading, const WaypointFilterData &filter)
{
  dest.clear();

  if (!filter.defined() && src.size() >= 500)
    return;

  FilterWaypointVisitor visitor(filter, location, heading, dest);

  if (filter.distance_index > 0)
    src.visit_within_range(location, Units::ToSysDistance(
        DistanceFilter[filter.distance_index]), visitor);
  else
    src.visit_name_prefix(filter.name, visitor);

  if (filter.distance_index > 0 || filter.direction_index > 0)
    std::sort(dest.begin(), dest.end(), WaypointDistanceCompare);
}

static void
FillLastUsedList(WaypointSelectInfoVector &dest,
                 const std::list<unsigned int> src, const Waypoints &waypoints,
                 GeoPoint location)
{
  dest.clear();

  if (src.empty())
    return;

  for (std::list<unsigned int>::const_reverse_iterator it = src.rbegin();
       it != src.rend(); it++) {
    const Waypoint* wp = waypoints.lookup_id(*it);
    if (wp == NULL)
      continue;

    dest.push_back(*wp, location);
  }
}

static void
UpdateList()
{
  if (filter_data.type_index == tfLastUsed)
    FillLastUsedList(WaypointSelectInfo, LastUsedWaypointNames,
                     way_points, g_location);
  else
    FillList(WaypointSelectInfo, way_points, g_location,
             last_heading, filter_data);

  wWaypointList->SetLength(std::max(1, (int)WaypointSelectInfo.size()));
  wWaypointList->SetOrigin(0);
  wWaypointList->SetCursorIndex(0);
  wWaypointList->invalidate();
}

static const TCHAR *
WaypointNameAllowedCharacters(const TCHAR *prefix)
{
  static TCHAR buffer[256];
  return way_points.suggest_name_prefix(prefix, buffer, ARRAY_SIZE(buffer));
}

static void
NameButtonUpdateChar()
{
  const TCHAR * NameFilter = WaypointNameAllowedCharacters(_T(""));
  if (NameFilterIdx == -1) {
    filter_data.name[0] = '\0';
    wbName->SetCaption(_T("*"));
  } else {
    filter_data.name[0] = NameFilter[NameFilterIdx];
    filter_data.name[1] = '\0';
    wbName->SetCaption(filter_data.name);
  }

  UpdateList();
}

static void
OnFilterNameButtonRight(gcc_unused WndButton &button)
{
  const TCHAR * NameFilter = WaypointNameAllowedCharacters(_T(""));
  NameFilterIdx++;
  if (NameFilterIdx > (int)(_tcslen(NameFilter) - 2))
    NameFilterIdx = -1;

  NameButtonUpdateChar();
}

static void
OnFilterNameButtonLeft(gcc_unused WndButton &button)
{
  const TCHAR * NameFilter = WaypointNameAllowedCharacters(_T(""));
  if (NameFilterIdx == -1)
    NameFilterIdx = (int)(_tcslen(NameFilter)-1);
  else
    NameFilterIdx--;

  NameButtonUpdateChar();
}

static void
OnFilterNameButton(gcc_unused WndButton &button)
{
  TCHAR newNameFilter[NAMEFILTERLEN + 1];
  CopyString(newNameFilter, filter_data.name, NAMEFILTERLEN + 1);
  dlgTextEntryShowModal(*(SingleWindow *)button.get_root_owner(),
                        newNameFilter, NAMEFILTERLEN, _("Waypoint name"),
                        WaypointNameAllowedCharacters);

  int i = _tcslen(newNameFilter) - 1;
  while (i >= 0) {
    if (newNameFilter[i] != _T(' '))
      break;

    newNameFilter[i] = 0;
    i--;
  }

  CopyString(filter_data.name, newNameFilter, NAMEFILTERLEN + 1);

  if (wbName) {
    if (string_is_empty(filter_data.name))
      wbName->SetCaption(_T("*"));
    else
      wbName->SetCaption(filter_data.name);
  }

  UpdateList();
}

static void
OnFilterDistance(DataField *Sender, DataField::DataAccessKind_t Mode)
{
  switch (Mode) {
  case DataField::daChange:
    filter_data.distance_index = Sender->GetAsInteger();
    UpdateList();
    break;

  case DataField::daSpecial:
    return;
  }
}

static void
OnFilterDirection(DataField *Sender, DataField::DataAccessKind_t Mode)
{
  switch (Mode) {
  case DataField::daChange:
    filter_data.direction_index = Sender->GetAsInteger();
    UpdateList();
    break;

  case DataField::daSpecial:
    return;
  }
}

static void
OnFilterType(DataField *Sender, DataField::DataAccessKind_t Mode)
{
  switch (Mode) {
  case DataField::daChange:
    filter_data.type_index = (type_filter)Sender->GetAsInteger();
    UpdateList();
    break;

  case DataField::daSpecial:
    return;
  }
}

static void
OnPaintListItem(Canvas &canvas, const PixelRect rc, unsigned i)
{
  if (WaypointSelectInfo.empty()) {
    assert(i == 0);

    const unsigned line_height = rc.bottom - rc.top;
    const Font &name_font = Fonts::MapBold;
    canvas.select(name_font);
    canvas.text(rc.left + line_height + Layout::FastScale(2),
                rc.top + line_height / 2 - name_font.get_height() / 2,
                filter_data.defined() || way_points.empty() ?
                _("No Match!") : _("Choose a filter or click here"));
    return;
  }

  assert(i < WaypointSelectInfo.size());

  const struct WaypointSelectInfo &info = WaypointSelectInfo[i];

  WaypointListRenderer::Draw(canvas, rc, *info.way_point,
                             GeoVector(info.Distance, info.Direction),
                             CommonInterface::main_window.look->waypoint,
                             CommonInterface::SettingsMap().waypoint);
}

static void
OnWaypointListEnter(gcc_unused unsigned i)
{
  if (WaypointSelectInfo.size() > 0)
    wf->SetModalResult(mrOK);
  else
    OnFilterNameButton(*wbName);
}

static void
OnWPSSelectClicked(gcc_unused WndButton &button)
{
  OnWaypointListEnter(0);
}

static void
OnWPSCloseClicked(gcc_unused WndButton &button)
{
  wf->SetModalResult(mrCancel);
}

static void
OnTimerNotify(gcc_unused WndForm &Sender)
{
  if (filter_data.direction_index == 1 && !XCSoarInterface::Calculated().circling) {
    Angle a = last_heading - CommonInterface::Calculated().heading;
    if (a.as_delta().magnitude_degrees() >= fixed(60)) {
      last_heading = CommonInterface::Calculated().heading;
      UpdateList();
      InitializeDirection(true);
      wpDirection->RefreshDisplay();
    }
  }
}

#ifdef GNAV

static bool
FormKeyDown(WndForm &Sender, unsigned key_code)
{
  type_filter NewIndex = filter_data.type_index;

  switch (key_code) {
  case VK_F1:
    NewIndex = tfAll;
    break;

  case VK_F2:
    NewIndex = tfLandable;
    break;

  case VK_F3:
    NewIndex = tfTurnpoint;
    break;

  default:
    return false;
  }

  if (filter_data.type_index != NewIndex) {
    filter_data.type_index = NewIndex;
    UpdateList();
    wpType->GetDataField()->SetAsInteger(filter_data.type_index);
    wpType->RefreshDisplay();
  }

  return true;
}

#endif /* GNAV */

static CallBackTableEntry CallBackTable[] = {
  DeclareCallBackEntry(OnFilterDistance),
  DeclareCallBackEntry(OnFilterDirection),
  DeclareCallBackEntry(OnFilterType),
  DeclareCallBackEntry(NULL)
};

void
dlgWaypointSelectAddToLastUsed(const Waypoint &wp)
{
  LastUsedWaypointNames.remove(wp.id);
  LastUsedWaypointNames.push_back(wp.id);
}

const Waypoint*
dlgWaypointSelect(SingleWindow &parent, const GeoPoint &location,
                  OrderedTask *ordered_task,
                  const unsigned ordered_task_index)
{
  wf = LoadDialog(CallBackTable, parent, Layout::landscape ?
      _T("IDR_XML_WAYPOINTSELECT_L") : _T("IDR_XML_WAYPOINTSELECT"));

  if (!wf)
    return NULL;

  assert(wf != NULL);

#ifdef GNAV
  wf->SetKeyDownNotify(FormKeyDown);
#endif

  ((WndButton *)wf->FindByName(_T("cmdClose")))->
      SetOnClickNotify(OnWPSCloseClicked);

  ((WndButton *)wf->FindByName(_T("cmdSelect")))->
      SetOnClickNotify(OnWPSSelectClicked);

  ((WndButton *)wf->FindByName(_T("cmdFltName")))->
      SetOnClickNotify(OnFilterNameButton);

  ((WndButton *)wf->FindByName(_T("cmdFltName")))->
      SetOnLeftNotify(OnFilterNameButtonLeft);

  ((WndButton *)wf->FindByName(_T("cmdFltName")))->
      SetOnRightNotify(OnFilterNameButtonRight);

  wWaypointList = (WndListFrame*)wf->FindByName(_T("frmWaypointList"));
  assert(wWaypointList != NULL);
  wWaypointList->SetActivateCallback(OnWaypointListEnter);
  wWaypointList->SetPaintItemCallback(OnPaintListItem);
  unsigned line_height = Fonts::MapBold.get_height() + Layout::Scale(6) +
                         Fonts::MapLabel.get_height();
  wWaypointList->SetItemHeight(line_height);

  wbName = (WndButton*)wf->FindByName(_T("cmdFltName"));
  wpDistance = (WndProperty*)wf->FindByName(_T("prpFltDistance"));
  wpDirection = (WndProperty*)wf->FindByName(_T("prpFltDirection"));
  wpType = (WndProperty *)wf->FindByName(_T("prpFltType"));

  g_location = location;
  FAITriPtVali = new FAITrianglePointValidator(ordered_task,
                                               ordered_task_index);
  last_heading = CommonInterface::Calculated().heading;

  PrepareData();
  UpdateList();

  wf->SetTimerNotify(OnTimerNotify);

  if (wf->ShowModal() != mrOK) {
    delete wf;
    delete FAITriPtVali;
    return NULL;
  }

  unsigned ItemIndex = wWaypointList->GetCursorIndex();

  delete wf;
  delete FAITriPtVali;

  const Waypoint* retval = NULL;

  if (ItemIndex < WaypointSelectInfo.size())
    retval = WaypointSelectInfo[ItemIndex].way_point;

  if (retval != NULL)
    dlgWaypointSelectAddToLastUsed(*retval);

  return retval;
}
