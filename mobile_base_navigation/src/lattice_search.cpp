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

#include "mobile_base_navigation/lattice_search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>

namespace mobile_base_navigation
{

namespace
{

constexpr double kQuarterPi = 0.78539816339744827900;  // pi / 4
constexpr double kTwoPi = 6.28318530717958623200;
constexpr double kRoot = 0.70710678118654752440;  // sqrt(1/2)

// Exact unit components per heading. Written out rather than computed with
// cos()/sin() so a diagonal is bit-for-bit |x| == |y|: the controller's
// "exactly one primitive" invariant is asserted with a tolerance, but there
// is no reason to spend that tolerance on trigonometry rounding.
constexpr double kCos[kNumHeadings] = {1.0, kRoot, 0.0, -kRoot, -1.0, -kRoot, 0.0, kRoot};
constexpr double kSin[kNumHeadings] = {0.0, kRoot, 1.0, kRoot, 0.0, -kRoot, -1.0, -kRoot};

// Cell deltas per heading, in the same order.
constexpr int kDx[kNumHeadings] = {1, 1, 0, -1, -1, -1, 0, 1};
constexpr int kDy[kNumHeadings] = {0, 1, 1, 1, 0, -1, -1, -1};

int wrapHeading(int heading)
{
  return ((heading % kNumHeadings) + kNumHeadings) % kNumHeadings;
}

int signOf(int value)
{
  return (value > 0) - (value < 0);
}

}  // namespace

double headingAngle(int heading)
{
  return wrapHeading(heading) * kQuarterPi;
}

double headingCos(int heading)
{
  return kCos[wrapHeading(heading)];
}

double headingSin(int heading)
{
  return kSin[wrapHeading(heading)];
}

bool isDiagonalHeading(int heading)
{
  return wrapHeading(heading) % 2 == 1;
}

int snapHeading(double yaw)
{
  double normalized = std::fmod(yaw, kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }
  return wrapHeading(static_cast<int>(std::llround(normalized / kQuarterPi)));
}

int snapHeadingWithHysteresis(double yaw, int previous, double hysteresis_rad)
{
  const int candidate = snapHeading(yaw);
  if (previous < 0 || candidate == wrapHeading(previous)) {
    return candidate;
  }

  // Give up the previous heading only once the yaw has moved clear of the
  // boundary by the hysteresis band. Without this a yaw parked on a 22.5
  // degree boundary alternates between two headings on consecutive replans,
  // and the first segment of the plan alternates with it.
  const int kept = wrapHeading(previous);
  double error = std::fabs(std::remainder(yaw - headingAngle(kept), kTwoPi));
  if (error <= kQuarterPi / 2.0 + hysteresis_rad) {
    return kept;
  }
  return candidate;
}

LatticeSearch::LatticeSearch(const LatticeSettings & settings)
: settings_(settings)
{
}

void LatticeSearch::setSettings(const LatticeSettings & settings)
{
  settings_ = settings;
}

int LatticeSearch::cellStep(const nav2_costmap_2d::Costmap2D & costmap) const
{
  const double resolution = costmap.getResolution();
  if (resolution <= 0.0) {
    return 1;
  }
  return std::max(1, static_cast<int>(std::lround(settings_.step_size_m / resolution)));
}

void LatticeSearch::buildBlockedMask(const nav2_costmap_2d::Costmap2D & costmap)
{
  size_x_ = costmap.getSizeInCellsX();
  size_y_ = costmap.getSizeInCellsY();
  blocked_.assign(static_cast<std::size_t>(size_x_) * size_y_, false);
  if (size_x_ == 0 || size_y_ == 0) {
    return;
  }

  const double resolution = costmap.getResolution();
  const int radius_cells = std::max(
    0, static_cast<int>(std::ceil(settings_.robot_radius_m / resolution)));
  const double radius_sq = settings_.robot_radius_m * settings_.robot_radius_m;

  // Offsets of the disc, precomputed once so the dilation below is a flat
  // scan rather than a nested distance test per cell pair.
  std::vector<std::pair<int, int>> disc;
  disc.reserve(static_cast<std::size_t>(2 * radius_cells + 1) * (2 * radius_cells + 1));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double wx = dx * resolution;
      const double wy = dy * resolution;
      if (wx * wx + wy * wy <= radius_sq) {
        disc.emplace_back(dx, dy);
      }
    }
  }

  for (unsigned int my = 0; my < size_y_; ++my) {
    for (unsigned int mx = 0; mx < size_x_; ++mx) {
      const unsigned char cost = costmap.getCost(mx, my);
      const bool lethal = cost == nav2_costmap_2d::LETHAL_OBSTACLE;
      const bool unknown =
        !settings_.allow_unknown && cost == nav2_costmap_2d::NO_INFORMATION;
      if (!lethal && !unknown) {
        continue;
      }
      // Every cell whose disc touches this obstacle is a pose the robot
      // cannot occupy, which is the sweep test stated as a dilation.
      for (const auto & offset : disc) {
        const int nx = static_cast<int>(mx) + offset.first;
        const int ny = static_cast<int>(my) + offset.second;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x_) ||
          ny >= static_cast<int>(size_y_))
        {
          continue;
        }
        blocked_[static_cast<std::size_t>(ny) * size_x_ + nx] = true;
      }
    }
  }
}

bool LatticeSearch::isBlocked(unsigned int mx, unsigned int my) const
{
  if (mx >= size_x_ || my >= size_y_) {
    return true;
  }
  return blocked_[static_cast<std::size_t>(my) * size_x_ + mx];
}

bool LatticeSearch::segmentClear(
  unsigned int from_mx, unsigned int from_my,
  unsigned int to_mx, unsigned int to_my) const
{
  // Only axis-aligned and exactly-diagonal segments are ever generated, so
  // stepping by the sign of each delta visits precisely the cells the robot
  // centre passes through. This is not a general line rasteriser.
  const int dx = static_cast<int>(to_mx) - static_cast<int>(from_mx);
  const int dy = static_cast<int>(to_my) - static_cast<int>(from_my);
  const int steps = std::max(std::abs(dx), std::abs(dy));
  const int sx = signOf(dx);
  const int sy = signOf(dy);

  for (int k = 1; k <= steps; ++k) {
    const int nx = static_cast<int>(from_mx) + sx * k;
    const int ny = static_cast<int>(from_my) + sy * k;
    if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x_) ||
      ny >= static_cast<int>(size_y_))
    {
      return false;
    }
    if (isBlocked(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))) {
      return false;
    }
  }
  return true;
}

LatticeResult LatticeSearch::search(
  const nav2_costmap_2d::Costmap2D & costmap,
  double start_x, double start_y, int start_heading,
  double goal_x, double goal_y,
  const std::function<bool()> & cancel_checker)
{
  LatticeResult result;
  buildBlockedMask(costmap);

  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  if (!costmap.worldToMap(start_x, start_y, start_mx, start_my)) {
    result.message = "start is outside the costmap";
    return result;
  }
  if (!costmap.worldToMap(goal_x, goal_y, goal_mx, goal_my)) {
    result.message = "goal is outside the costmap";
    return result;
  }
  if (isBlocked(start_mx, start_my)) {
    result.message = "start is occupied: the robot footprint overlaps an obstacle";
    return result;
  }

  const double resolution = costmap.getResolution();
  const int step = cellStep(costmap);
  const double cardinal_length = resolution * step;
  const double diagonal_length = cardinal_length * std::sqrt(2.0);
  const double rotate_cost = settings_.turning_cost_weight * kQuarterPi;
  // Half a cell diagonal, so the goal is never unreachable purely because it
  // sits between cell centres.
  const double arrival = std::max(settings_.tolerance_m, resolution * 0.7072);
  const double arrival_sq = arrival * arrival;

  const std::size_t cells = static_cast<std::size_t>(size_x_) * size_y_;
  const std::size_t states = cells * kNumHeadings;
  constexpr float kInfinity = std::numeric_limits<float>::max();
  std::vector<float> g(states, kInfinity);
  std::vector<std::int64_t> parent(states, -1);
  std::vector<bool> closed(states, false);

  auto index_of = [this](unsigned int mx, unsigned int my, int heading) {
      return (static_cast<std::size_t>(my) * size_x_ + mx) * kNumHeadings + heading;
    };

  const int start_h = wrapHeading(start_heading);
  const std::size_t start_index = index_of(start_mx, start_my, start_h);

  auto heuristic = [&](unsigned int mx, unsigned int my) {
      double wx = 0.0;
      double wy = 0.0;
      costmap.mapToWorld(mx, my, wx, wy);
      return std::hypot(wx - goal_x, wy - goal_y);
    };

  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  g[start_index] = 0.0f;
  open.emplace(heuristic(start_mx, start_my), start_index);

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(settings_.max_planning_time_s);

  // Checked once up front as well as periodically below. A short search over
  // open floor finishes inside the periodic interval, so without this a goal
  // cancelled before the search began would still return a path.
  if (cancel_checker && cancel_checker()) {
    result.message = "planning cancelled";
    return result;
  }
  std::size_t goal_index = states;

  while (!open.empty()) {
    const std::size_t current = open.top().second;
    open.pop();
    if (closed[current]) {
      continue;
    }
    closed[current] = true;
    ++result.expansions;

    if ((result.expansions & 0xFFu) == 0u) {
      if (std::chrono::steady_clock::now() > deadline) {
        result.message = "planning time budget exhausted";
        return result;
      }
      if (cancel_checker && cancel_checker()) {
        result.message = "planning cancelled";
        return result;
      }
    }

    const int heading = static_cast<int>(current % kNumHeadings);
    const std::size_t cell = current / kNumHeadings;
    const unsigned int mx = static_cast<unsigned int>(cell % size_x_);
    const unsigned int my = static_cast<unsigned int>(cell / size_x_);

    double wx = 0.0;
    double wy = 0.0;
    costmap.mapToWorld(mx, my, wx, wy);
    const double dx_goal = wx - goal_x;
    const double dy_goal = wy - goal_y;
    if (dx_goal * dx_goal + dy_goal * dy_goal <= arrival_sq) {
      goal_index = current;
      break;
    }

    // ROTATE, plus and minus one heading. Restricting rotation to adjacent
    // headings keeps the branching factor at three; because the cost is
    // linear in angle, composing two 45 degree edges costs exactly what one
    // 90 degree edge would, so nothing is lost.
    for (const int turn : {-1, 1}) {
      const int next_heading = wrapHeading(heading + turn);
      const std::size_t next = index_of(mx, my, next_heading);
      if (closed[next]) {
        continue;
      }
      const double tentative = g[current] + rotate_cost;
      if (tentative < g[next]) {
        g[next] = static_cast<float>(tentative);
        parent[next] = static_cast<std::int64_t>(current);
        open.emplace(tentative + heuristic(mx, my), next);
      }
    }

    // TRANSLATE, along the current heading only. A diagonal is its own edge,
    // not a forward edge composed with a strafe: the robot executes it as a
    // single primitive, so the search has to cost it as one.
    const int nx = static_cast<int>(mx) + kDx[heading] * step;
    const int ny = static_cast<int>(my) + kDy[heading] * step;
    if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x_) ||
      ny >= static_cast<int>(size_y_))
    {
      continue;
    }
    const unsigned int tx = static_cast<unsigned int>(nx);
    const unsigned int ty = static_cast<unsigned int>(ny);
    if (!segmentClear(mx, my, tx, ty)) {
      continue;
    }
    const std::size_t next = index_of(tx, ty, heading);
    if (closed[next]) {
      continue;
    }
    const double length = isDiagonalHeading(heading) ? diagonal_length : cardinal_length;
    const double cost = static_cast<double>(costmap.getCost(tx, ty));
    const double penalty = settings_.cost_penalty_weight *
      std::min(cost, 252.0) / 252.0;
    const double tentative = g[current] + length * (1.0 + penalty);
    if (tentative < g[next]) {
      g[next] = static_cast<float>(tentative);
      parent[next] = static_cast<std::int64_t>(current);
      open.emplace(tentative + heuristic(tx, ty), next);
    }
  }

  if (goal_index >= states) {
    result.message = "no valid path: the goal is unreachable under the motion primitives";
    return result;
  }

  std::vector<LatticeState> raw;
  for (std::int64_t node = static_cast<std::int64_t>(goal_index); node >= 0;
    node = parent[static_cast<std::size_t>(node)])
  {
    const std::size_t current = static_cast<std::size_t>(node);
    const int heading = static_cast<int>(current % kNumHeadings);
    const std::size_t cell = current / kNumHeadings;
    LatticeState state;
    state.heading = heading;
    costmap.mapToWorld(
      static_cast<unsigned int>(cell % size_x_),
      static_cast<unsigned int>(cell / size_x_),
      state.x, state.y);
    raw.push_back(state);
  }
  std::reverse(raw.begin(), raw.end());

  // Merge, in two passes. First drop the interior of every straight run, so a
  // 40-cell corridor is two poses rather than 41 and the controller sees
  // segments instead of a bead chain.
  std::vector<LatticeState> corners;
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const bool first = i == 0;
    const bool last = i + 1 == raw.size();
    if (first || last ||
      raw[i].heading != raw[i + 1].heading ||
      raw[i].heading != raw[i - 1].heading)
    {
      corners.push_back(raw[i]);
    }
  }

  // Then collapse the stack of states a multi-step rotation leaves at one
  // position, keeping the last so the surviving heading is the outgoing one.
  result.waypoints.clear();
  for (const auto & state : corners) {
    if (!result.waypoints.empty() &&
      std::fabs(result.waypoints.back().x - state.x) < 1e-9 &&
      std::fabs(result.waypoints.back().y - state.y) < 1e-9)
    {
      result.waypoints.back().heading = state.heading;
      continue;
    }
    result.waypoints.push_back(state);
  }

  result.success = true;
  result.message = "ok";
  return result;
}

}  // namespace mobile_base_navigation
