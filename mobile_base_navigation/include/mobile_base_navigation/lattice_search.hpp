// Copyright 2026 Safwan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOBILE_BASE_NAVIGATION__LATTICE_SEARCH_HPP_
#define MOBILE_BASE_NAVIGATION__LATTICE_SEARCH_HPP_

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace mobile_base_navigation
{

/// The lattice admits exactly eight travel directions, 45 degrees apart.
constexpr int kNumHeadings = 8;

/// Centre-to-centre angle of a heading index, in radians, in [0, 2*pi).
double headingAngle(int heading);

/// Nearest heading index to an arbitrary yaw. Ties round away from zero.
int snapHeading(double yaw);

/**
 * Nearest heading index, but sticky.
 *
 * A yaw sitting near a 22.5 degree boundary snaps to whichever side noise
 * pushes it towards, and the resulting plan's first segment flips between two
 * primitives on consecutive replans. `hysteresis_rad` is how much better the
 * new candidate has to be before the previous heading is given up. Pass
 * `previous < 0` when there is no previous heading to keep.
 */
int snapHeadingWithHysteresis(double yaw, int previous, double hysteresis_rad);

/// Unit x component of a heading. Exactly -1, 0, 1 or +/- sqrt(1/2).
double headingCos(int heading);

/// Unit y component of a heading. Exactly -1, 0, 1 or +/- sqrt(1/2).
double headingSin(int heading);

/// True for the four 45-degree headings, which move in x and y together.
bool isDiagonalHeading(int heading);

/// Tuning shared by the search core and the plugin that wraps it.
struct LatticeSettings
{
  /// Length of one TRANSLATE edge. Quantised up to a whole number of cells at
  /// search time: a diagonal edge that did not land on a cell centre would
  /// leave the lattice after a few expansions.
  double step_size_m{0.05};

  /// Multiplies the angular distance of a ROTATE edge, in metres per radian,
  /// so it is directly comparable with a TRANSLATE edge's length. Raising it
  /// buys longer straight and diagonal runs at the price of a longer route.
  double turning_cost_weight{0.35};

  /// Scales a TRANSLATE edge's length by (1 + weight * cost / 252) so the
  /// search prefers to stay out of inflated space. Must stay >= 0: a negative
  /// weight would make an edge cheaper than its own length and break the
  /// admissibility of the Euclidean heuristic.
  double cost_penalty_weight{2.0};

  /// The robot's circumscribed radius. Every cell the disc of this radius
  /// sweeps along an edge must be clear for the edge to be valid.
  double robot_radius_m{0.14};

  /// How close to the goal a lattice cell must be to count as arrival.
  double tolerance_m{0.125};

  /// Nav2's planner.yaml keeps this false; unknown space is then as
  /// impassable as a wall.
  bool allow_unknown{false};

  /// Search budget. A* over (x, y, heading) is fast, but a goal enclosed by
  /// obstacles exhausts the whole reachable set before it can fail.
  double max_planning_time_s{2.0};
};

/// One lattice waypoint, in world coordinates, with its outgoing heading.
struct LatticeState
{
  double x{0.0};
  double y{0.0};
  int heading{0};
};

/// Outcome of one search.
struct LatticeResult
{
  bool success{false};
  /// Waypoints after consecutive same-heading edges have been merged, so a
  /// straight run of 40 cells is two poses and not 41.
  std::vector<LatticeState> waypoints;
  std::string message;
  std::size_t expansions{0};
};

/**
 * A* over (x, y, heading) restricted to this robot's motion primitives.
 *
 * Deliberately free of ROS types beyond `nav2_costmap_2d::Costmap2D`, so the
 * edge generation and heading snapping can be tested against a synthetic
 * costmap with no node, no lifecycle and no simulator.
 */
class LatticeSearch
{
public:
  LatticeSearch() = default;
  explicit LatticeSearch(const LatticeSettings & settings);

  void setSettings(const LatticeSettings & settings);
  const LatticeSettings & settings() const {return settings_;}

  /**
   * Dilate the costmap's lethal set by the robot radius.
   *
   * Done once per search rather than per edge: the naive "sweep the disc
   * along every candidate edge" check is O(states * edges * disc), which is
   * tens of millions of lookups on a room-sized map, while dilating first is
   * O(cells * disc) and turns every later edge check into one lookup.
   *
   * Only LETHAL_OBSTACLE is dilated, never INSCRIBED_INFLATED_OBSTACLE. The
   * inflation layer has already marked the inscribed band using the same
   * robot_radius, so dilating that band as well would apply the footprint
   * twice and close gaps the robot fits through.
   */
  void buildBlockedMask(const nav2_costmap_2d::Costmap2D & costmap);

  /// True if the robot's disc centred on this cell touches anything lethal.
  bool isBlocked(unsigned int mx, unsigned int my) const;

  /// True if every cell stepped through from one cell to another is clear.
  bool segmentClear(
    unsigned int from_mx, unsigned int from_my,
    unsigned int to_mx, unsigned int to_my) const;

  /// Number of costmap cells in one TRANSLATE edge, at least one.
  int cellStep(const nav2_costmap_2d::Costmap2D & costmap) const;

  LatticeResult search(
    const nav2_costmap_2d::Costmap2D & costmap,
    double start_x, double start_y, int start_heading,
    double goal_x, double goal_y,
    const std::function<bool()> & cancel_checker = {});

private:
  LatticeSettings settings_;
  std::vector<bool> blocked_;
  unsigned int size_x_{0};
  unsigned int size_y_{0};
};

}  // namespace mobile_base_navigation

#endif  // MOBILE_BASE_NAVIGATION__LATTICE_SEARCH_HPP_
